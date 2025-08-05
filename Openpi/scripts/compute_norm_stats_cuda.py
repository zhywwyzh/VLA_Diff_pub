# Monkey-patch to fix 'List' feature type error in old datasets
try:
    import datasets.features.features as features

    _OLD_GENERATE_FROM_DICT = features.generate_from_dict

    def _new_generate_from_dict(obj):
        if isinstance(obj, dict) and obj.get("_type") == "List":
            obj["_type"] = "Sequence"
        return _OLD_GENERATE_FROM_DICT(obj)

    features.generate_from_dict = _new_generate_from_dict
except (ImportError, AttributeError):
    # If datasets or the function doesn't exist, do nothing.
    pass
# End of monkey-patch

"""Compute normalization statistics for a config using CUDA for acceleration.

This script is used to compute the normalization statistics for a given config. It
will compute the mean and standard deviation of the data in the dataset and save it
to the config assets directory.
"""

import numpy as np
import torch
import tqdm
import tyro

import openpi.shared.normalize as normalize
import openpi.training.config as _config
import openpi.training.data_loader as _data_loader
import openpi.transforms as transforms


class RemoveStrings(transforms.DataTransformFn):
    def __call__(self, x: dict) -> dict:
        return {k: v for k, v in x.items() if not np.issubdtype(np.asarray(v).dtype, np.str_)}


def create_dataset(config: _config.TrainConfig) -> tuple[_config.DataConfig, _data_loader.Dataset]:
    data_config = config.data.create(config.assets_dirs, config.model)
    if data_config.repo_id is None:
        raise ValueError("Data config must have a repo_id")
    dataset = _data_loader.create_dataset(data_config, config.model)
    dataset = _data_loader.TransformedDataset(
        dataset,
        [
            *data_config.repack_transforms.inputs,
            *data_config.data_transforms.inputs,
            RemoveStrings(),
        ],
    )
    return data_config, dataset


def main(
    config_name: str,
    batch_size: int = 256,
    max_frames: int | None = None,
    device: str = "cuda:0",
):
    if not torch.cuda.is_available():
        raise RuntimeError("CUDA is not available, this script requires a GPU.")
    
    torch_device = torch.device(device)

    config = _config.get_config(config_name)
    data_config, dataset = create_dataset(config)

    num_frames = len(dataset)
    shuffle = False

    max_frames = num_frames // 5

    if max_frames is not None and max_frames < num_frames:
        num_frames = max_frames
        shuffle = True

    num_batches = (num_frames + batch_size - 1) // batch_size

    data_loader = _data_loader.TorchDataLoader(
        dataset,
        local_batch_size=batch_size,
        num_workers=8,
        shuffle=shuffle,
        num_batches=num_batches,
    )

    keys = ["state", "actions"]
    sums = {key: None for key in keys}
    sum_sqs = {key: None for key in keys}
    counts = {key: 0 for key in keys}

    for batch in tqdm.tqdm(data_loader, total=num_batches, desc="Computing stats on GPU"):
        for key in keys:
            if key not in batch:
                continue
            
            values = torch.from_numpy(np.asarray(batch[key])).to(torch_device, non_blocking=True).float()
            values = values.reshape(-1, values.shape[-1])
            
            if sums[key] is None:
                feature_dim = values.shape[-1]
                sums[key] = torch.zeros(feature_dim, device=torch_device, dtype=torch.float64)
                sum_sqs[key] = torch.zeros(feature_dim, device=torch_device, dtype=torch.float64)

            sums[key] += values.sum(dim=0)
            sum_sqs[key] += (values**2).sum(dim=0)
            counts[key] += values.shape[0]

    norm_stats = {}
    for key in keys:
        if counts[key] > 0:
            mean = sums[key] / counts[key]
            std = torch.sqrt(sum_sqs[key] / counts[key] - mean**2)
            norm_stats[key] = {
                "mean": mean.cpu().numpy(),
                "std": std.cpu().numpy(),
            }

    output_path = config.assets_dirs / data_config.repo_id
    print(f"Writing stats to: {output_path}")
    normalize.save(output_path, norm_stats)


if __name__ == "__main__":
    tyro.cli(main)
