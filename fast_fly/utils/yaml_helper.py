import yaml
from fast_fly.utils.logger import getLogger
from typing import Optional, Any

class ConfigReader:
    def __init__(self, yaml_path) -> None:
        self.logger = getLogger("ConfigReader")
        with open(yaml_path, 'r') as f:
            self.yaml_cfg = yaml.load(f, Loader=yaml.FullLoader)
        self.logger.info(f"load params from file {yaml_path}")

    def load_param(self, param_name, default: Optional[Any] = None):
        if param_name in self.yaml_cfg:
            value = self.yaml_cfg[param_name]
            self.logger.info(f"Read value from file {param_name} : {value}")

        elif default is not None:
            value = default
            self.logger.warn(f"Using default value for {param_name} : {default}")  # noqa
        else:
            self.logger.error(
                f"Parameter {param_name} not found in config file")
            raise ValueError(
                f"Parameter {param_name} not found in config file")
        return value

