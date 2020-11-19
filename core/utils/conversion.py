# 参考：Power = Speed Times Force 总结版！.afx
import math
from dataclasses import dataclass

@dataclass
class ElectricMachinery:
    NUMBER_OF_POLE_PAIRS : int
    RATED_CURRENT_RMS : float
    RATED_POWER_WATT : float
    RATED_SPEED_RPM : float 
    name : str = 'nameless'

    def __post_init__(self):

        RATED_RATED_SPEED_RPS = self.RATED_SPEED_RPM / 60 
        RATED_RATED_SPEED_RAD_PER_SEC = RATED_RATED_SPEED_RPS * (2*math.pi)

        RATED_TORQUE = self.RATED_POWER_WATT / RATED_RATED_SPEED_RAD_PER_SEC
        TORQUE_CONSTANT_NM_PER_ARMS  = RATED_TORQUE / (self.RATED_CURRENT_RMS)
        TORQUE_CONSTANT_NM_PER_APEAK = RATED_TORQUE / (self.RATED_CURRENT_RMS*1.414)

        # unit is volt sec and the sec here is in fact 1 / (rad/sec). Here, rad/sec means electrical speed.
        AMPL_INVARIANT_BACK_EMF_CONSTANT_VS  = AMPL_INVARIANT_PM_FLUX_LINKAGE = TORQUE_CONSTANT_NM_PER_APEAK / 1.5 / self.NUMBER_OF_POLE_PAIRS
        POWER_INVARIANT_BACK_EMF_CONSTANT_VS = TORQUE_CONSTANT_NM_PER_APEAK / self.NUMBER_OF_POLE_PAIRS

        AMPL_INVARIANT_BACK_EMF_CONSTANT_VrmsS  = AMPL_INVARIANT_PM_FLUX_LINKAGE = TORQUE_CONSTANT_NM_PER_ARMS / 1.5 / self.NUMBER_OF_POLE_PAIRS

        # Convert electrical speed rad/sec to mechanical speed r/min
        AMPL_INVARIANT_BACK_EMF_CONSTANT_MV_PER_RPM    = AMPL_INVARIANT_BACK_EMF_CONSTANT_VS    * 1e3 / (1.0/self.NUMBER_OF_POLE_PAIRS/2/3.1415926*60)
        AMPL_INVARIANT_BACK_EMF_CONSTANT_MVrms_PER_RPM = AMPL_INVARIANT_BACK_EMF_CONSTANT_VrmsS * 1e3 / (1.0/self.NUMBER_OF_POLE_PAIRS/2/3.1415926*60)

        # Valid for linear motor
        ELECTRICAL_NORTH_NORTH_PITCH_MM = 30
        MECHANICAL_ONE_CYCLE_TRAVLE_MM = ELECTRICAL_NORTH_NORTH_PITCH_MM * self.NUMBER_OF_POLE_PAIRS
        RATED_SPEED_MM_PER_SEC = self.RATED_SPEED_RPM / 60 * (ELECTRICAL_NORTH_NORTH_PITCH_MM*self.NUMBER_OF_POLE_PAIRS)
        RADIUS_MM = MECHANICAL_ONE_CYCLE_TRAVLE_MM / (2*math.pi)
        RATED_FORCE = RATED_TORQUE / (RADIUS_MM*1e-3)
        # FORCE_CONSTANT_N_PER_ARMS
        # FORCE_CONSTANT_N_PER_APEAK

        print('---------\n',
        f'{RATED_TORQUE:.4f} Nm\n',
        f'{TORQUE_CONSTANT_NM_PER_ARMS:.3f} Nm/Arms\n',
        f'{AMPL_INVARIANT_BACK_EMF_CONSTANT_MV_PER_RPM:.3f} mV/rpm\n',
        f'{AMPL_INVARIANT_BACK_EMF_CONSTANT_MVrms_PER_RPM:.3f} mVrms/rpm\n',
        f'{RATED_SPEED_MM_PER_SEC} mm/s\n',
        f'{AMPL_INVARIANT_BACK_EMF_CONSTANT_VS:g}, Vs',
            sep='\t')

        # quit()

        # print('---------\n',
        # f'{RATED_TORQUE:.4f}', 'Nm\n',
        # f'{RATED_FORCE:.2f}', 'N\n',
        # f'{TORQUE_CONSTANT_NM_PER_ARMS:.3f}', 'Nm/Arms\n',
        # f'{AMPL_INVARIANT_BACK_EMF_CONSTANT_MV_PER_RPM:.3f}', 'mV/rpm\n',
        # f'{AMPL_INVARIANT_BACK_EMF_CONSTANT_MVrms_PER_RPM:.3f}', 'mVrms/rpm\n',
        # RATED_SPEED_MM_PER_SEC, 'mm/s\n',
        #     sep='\t')

if __name__ == '__main__':
    em = ElectricMachinery( NUMBER_OF_POLE_PAIRS = 4,
                                       RATED_CURRENT_RMS = 12.8,
                                       RATED_POWER_WATT = 400,
                                       RATED_SPEED_RPM = 3000
        )
