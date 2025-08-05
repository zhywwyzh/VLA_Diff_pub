import jax
import flax.nnx as nnx
from openpi.training import config as train_config
from openpi.shared import nnx_utils

def main():
    # 1. 获取基础模型的训练配置
    config_name = "pi0_uav_low_mem_finetune"
    config = train_config.get_config(config_name)

    print(f"Inspecting model structure for config: '{config_name}'")
    print(f"PaliGemma Variant: {config.model.paligemma_variant}")
    print(f"Action Expert Variant: {config.model.action_expert_variant}")
    print("-" * 50)

    # 2. 创建模型实例
    # 我们只需要形状信息，所以用 jax.eval_shape 来避免实际的权重计算
    def create_model_state():
        rng = jax.random.key(0)
        model = config.model.create(rng)
        return nnx.state(model)

    params = jax.eval_shape(create_model_state)

    # 3. 打印所有参数的路径
    print("All parameter paths in the model:")
    param_paths = nnx_utils.get_pytree_path_names(params)
    for path in param_paths:
        print(path)

    print("-" * 50)

    # 4. 专门找出包含 "lora" 的路径
    print("Paths containing 'lora':")
    lora_paths = [path for path in param_paths if "lora" in path]
    if lora_paths:
        for path in lora_paths:
            print(path)
    else:
        print("No 'lora' paths found.")

if __name__ == "__main__":
    main()