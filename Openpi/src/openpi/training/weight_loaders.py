import dataclasses
import logging
import re
from typing import Protocol, runtime_checkable

import flax.traverse_util
import numpy as np

import openpi.models.model as _model
import openpi.shared.array_typing as at
import openpi.shared.download as download

logger = logging.getLogger(__name__)


@runtime_checkable
class WeightLoader(Protocol):
    def load(self, params: at.Params) -> at.Params:
        """Loads the model weights.

        Args:
            params: Parameters of the model. This is a nested structure of array-like objects that
                represent the model's parameters.

        Returns:
            Loaded parameters. The structure must be identical to `params`. If returning a subset of
            the parameters the loader must merge the loaded parameters with `params`.
        """


@dataclasses.dataclass(frozen=True)
class NoOpWeightLoader(WeightLoader):
    def load(self, params: at.Params) -> at.Params:
        return params


@dataclasses.dataclass(frozen=True)
class CheckpointWeightLoader(WeightLoader):
    """Loads an entire set of weights from a checkpoint.

    Compatible with:
      trained checkpoints:
        example: "./checkpoints/<config>/<exp>/<step>/params"
      released checkpoints:
        example: "s3://openpi-assets/checkpoints/<model>/params"
    """

    params_path: str

    def load(self, params: at.Params) -> at.Params:
        # We are loading np.ndarray and relying on the training code to properly convert and shard the params.
        loaded_params = _model.restore_params(download.maybe_download(self.params_path), restore_type=np.ndarray)
        # Add all missing LoRA weights.
        return _merge_params(loaded_params, params, missing_regex=".*lora.*")


@dataclasses.dataclass(frozen=True)
class PaliGemmaWeightLoader(WeightLoader):
    """Loads weights from the official PaliGemma checkpoint.

    This will overwrite existing weights with similar names while keeping all extra weights intact.
    This allows us to support the action expert which is used by the Pi0 model.
    """

    def load(self, params: at.Params) -> at.Params:
        path = download.maybe_download(
            "gs://vertex-model-garden-paligemma-us/paligemma/pt_224.npz", gs={"token": "anon"}
        )
        with path.open("rb") as f:
            flat_params = dict(np.load(f, allow_pickle=False))
        loaded_params = {"PaliGemma": flax.traverse_util.unflatten_dict(flat_params, sep="/")["params"]}
        # Add all missing weights.
        return _merge_params(loaded_params, params, missing_regex=".*")


@dataclasses.dataclass(frozen=True)
class Pi0BaseWithNativePaliGemmaLoader(WeightLoader):
    """Loads weights from the pi0_base checkpoint, but overwrites the PaliGemma
    part with the original pre-trained PaliGemma weights.
    """

    pi0_base_params_path: str = "s3://openpi-assets/checkpoints/pi0_base/params"

    def load(self, params: at.Params) -> at.Params:
        # 1. Load the pi0_base checkpoint which contains pre-trained action expert.
        # This returns a PyTree of actual np.ndarray weights.
        logger.info(f"Loading base weights from {self.pi0_base_params_path}")
        pi0_base_weights = _model.restore_params(
            download.maybe_download(self.pi0_base_params_path), restore_type=np.ndarray
        )

        # 2. Load native PaliGemma weights.
        # This also returns a PyTree of actual np.ndarray weights.
        logger.info("Loading native PaliGemma weights.")
        native_paligemma_loader = PaliGemmaWeightLoader()
        # We pass a dummy empty dict `{}` because we only want the loaded weights,
        # not the result of any merge operations yet.
        native_paligemma_weights = native_paligemma_loader.load({})

        # 3. Merge the two sets of loaded weights. The native PaliGemma weights
        # will overwrite the VLM part from the pi0_base checkpoint.
        # The `params` argument here is the model skeleton with ShapeDtypeStructs.
        merged_loaded_weights = _merge_params(native_paligemma_weights, pi0_base_weights, missing_regex=".*")

        # 4. Finally, merge the combined actual weights with the model skeleton
        # to add any missing parameters like LoRA layers.
        return _merge_params(merged_loaded_weights, params, missing_regex=".*lora.*")


def _merge_params(loaded_params: at.Params, params: at.Params, *, missing_regex: str) -> at.Params:
    """Merges the loaded parameters with the reference parameters.

    Args:
        loaded_params: The parameters to merge.
        params: The reference parameters.
        missing_regex: A regex pattern for all missing keys that should be merged from the reference parameters.

    Returns:
        A new dictionary with the merged parameters.
    """
    flat_ref = flax.traverse_util.flatten_dict(params, sep="/")
    flat_loaded = flax.traverse_util.flatten_dict(loaded_params, sep="/")

    # First, take all weights that are a subset of the reference weights.
    result = {}
    for k, v in flat_loaded.items():
        if k in flat_ref:
            result[k] = v.astype(flat_ref[k].dtype)

    # Then, merge any missing weights as defined by the missing regex.
    pattern = re.compile(missing_regex)
    for k in {k for k in flat_ref if pattern.fullmatch(k)}:
        if k not in result:
            result[k] = flat_ref[k]

    return flax.traverse_util.unflatten_dict(result, sep="/")