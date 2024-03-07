from fast_fly.utils.gate import GateConfig
import fast_fly
import os
package_path = os.path.dirname(fast_fly.__file__)


def test_gate():
    yaml_path=os.path.join(os.path.dirname(package_path),"fast_fly/config/gates_n7.yaml")
    gate_cfg=GateConfig(yaml_path)
    assert gate_cfg.n_gate==7

test_gate()