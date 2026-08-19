import os

SPARK_PIPELINE_ROOT = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))

from .base.base_pipeline_config import BasePipelineConfig
from .base.base_pipeline import BasePipeline
from .teleop.teleop_pipeline_config import TeleopPipelineConfig
from .teleop.teleop_pipeline import TeleopPipeline
from .autonomy.benchmark_pipeline_config import BenchmarkPipelineConfig
from .autonomy.unitree_g1_benchmark_pipeline_config import UnitreeG1BenchmarkPipelineConfig
from .autonomy.kinova_gen3_single_arm_benchmark_pipeline_config import (
    KinovaGen3SingleArmBenchmarkPipelineConfig,
)
from .autonomy.kinova_gen3_dual_arm_benchmark_pipeline_config import (
    KinovaGen3DualArmBenchmarkPipelineConfig,
)
from .autonomy.kuka_iiwa14_single_arm_benchmark_pipeline_config import (
    KukaIIWA14SingleArmBenchmarkPipelineConfig,
)
from .autonomy.fanuc_lrmate200id_single_arm_benchmark_pipeline_config import (
    FanucLRMate200iDSingleArmBenchmarkPipelineConfig,
)
from .autonomy.fanuc_lrmate200id_dual_arm_benchmark_pipeline_config import (
    FanucLRMate200iDDualArmBenchmarkPipelineConfig,
)
from .autonomy.galaxea_r1lite_fixed_base_benchmark_pipeline_config import (
    GalaxeaR1LiteFixedBaseBenchmarkPipelineConfig,
)
from .autonomy.agibot_g1_benchmark_pipeline_config import AgiBotG1BenchmarkPipelineConfig
from .teleop.kinova_gen3_single_arm_teleop_pipeline_config import (
    KinovaGen3SingleArmTeleopPipelineConfig,
)
from .teleop.kinova_gen3_dual_arm_teleop_pipeline_config import (
    KinovaGen3DualArmTeleopPipelineConfig,
)
from .teleop.kuka_iiwa14_single_arm_teleop_pipeline_config import (
    KukaIIWA14SingleArmTeleopPipelineConfig,
)
from .teleop.kuka_iiwa14_dual_arm_teleop_pipeline_config import (
    KukaIIWA14DualArmTeleopPipelineConfig,
)
from .teleop.fanuc_lrmate200id_single_arm_teleop_pipeline_config import (
    FanucLRMate200iDSingleArmTeleopPipelineConfig,
)
from .teleop.fanuc_lrmate200id_dual_arm_teleop_pipeline_config import (
    FanucLRMate200iDDualArmTeleopPipelineConfig,
)
from .teleop.galaxea_r1lite_fixed_base_teleop_pipeline_config import (
    GalaxeaR1LiteFixedBaseTeleopPipelineConfig,
)
from .teleop.galaxea_r1lite_right_arm_teleop_pipeline_config import (
    GalaxeaR1LiteRightArmTeleopPipelineConfig,
)
from .teleop.galaxea_r1lite_dual_arm_teleop_pipeline_config import (
    GalaxeaR1LiteDualArmTeleopPipelineConfig,
)
from .teleop.unitree_g1_teleop_pipeline_config import UnitreeG1TeleopPipelineConfig
from .teleop.unitree_g1_wbt_whole_body_teleop_pipeline_config import (
    UNITREE_G1_WBT_PRE_SAFE_CONTROL_NAMES,
    UnitreeG1WBTCascadeWholeBodyTeleopPipelineConfig,
    UnitreeG1WBTCascadeWholeBodyWithHandTeleopPipelineConfig,
)
from .teleop.unitree_g1_sonic_whole_body_teleop_pipeline_config import (
    UNITREE_G1_SONIC_PRE_SAFE_CONTROL_NAMES,
    UnitreeG1SonicCascadeWholeBodyTeleopPipelineConfig,
    UnitreeG1SonicCascadeWholeBodyWithHandTeleopPipelineConfig,
)
from .teleop.agibot_g1_teleop_pipeline_config import AgiBotG1TeleopPipelineConfig
from .teleop.agibot_g1_right_arm_teleop_pipeline_config import AgiBotG1RightArmTeleopPipelineConfig
from .teleop.agibot_g1_dual_arm_teleop_pipeline_config import AgiBotG1DualArmTeleopPipelineConfig
from .visualization import render_critical_pairs, render_value_based_debug_info
from .simulation_runtime import run_simulation_pipeline
from .autonomy.benchmark_test_case_generator import (
    LEGACY_TASK_CASE_LIST,
    generate_benchmark_test_case,
)
from .autonomy.benchmark_test_cases import (
    BenchmarkTestCase,
    apply_benchmark_test_case,
    list_benchmark_test_cases,
    resolve_benchmark_test_case,
)
