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

        this = self

        # Omega
        this.RATED_RATED_SPEED_REV_PER_SEC = self.RATED_SPEED_RPM / 60 
        this.RATED_RATED_SPEED_RAD_PER_SEC = this.RATED_RATED_SPEED_REV_PER_SEC * (2*math.pi)

        # Tem = KT * IN = PW / Omega
        this.RATED_TORQUE = self.RATED_POWER_WATT / this.RATED_RATED_SPEED_RAD_PER_SEC
        # KT = 1.5 * npp * KE = Tem / IN
        this.TORQUE_CONSTANT_Nm_PER_Arms  = this.RATED_TORQUE / (self.RATED_CURRENT_RMS)
        this.TORQUE_CONSTANT_Nm_PER_Apeak = this.RATED_TORQUE / (self.RATED_CURRENT_RMS*1.414)

        # KE = UN / (Omega*npp)
        # unit is volt*sec where 'sec' here is in fact 1 / (rad/sec)---do note rad/sec means electrical speed.
        this.AMPL_INVARIANT_BACK_EMF_CONSTANT_VS    = AMPL_INVARIANT_PM_FLUX_LINKAGE_Wb  = this.TORQUE_CONSTANT_Nm_PER_Apeak / 1.5 / self.NUMBER_OF_POLE_PAIRS
        this.AMPL_INVARIANT_BACK_EMF_CONSTANT_VrmsS                                      = this.AMPL_INVARIANT_BACK_EMF_CONSTANT_VS / 1.414
        this.POWER_INVARIANT_BACK_EMF_CONSTANT_VS   = POWER_INVARIANT_PM_FLUX_LINKAGE_Wb = this.TORQUE_CONSTANT_Nm_PER_Apeak / self.NUMBER_OF_POLE_PAIRS / math.sqrt(1.5)

        # KE
        # Convert electrical speed rad/sec to mechanical speed r/min
        this.AMPL_INVARIANT_BACK_EMF_CONSTANT_MV_PER_RPM    = this.AMPL_INVARIANT_BACK_EMF_CONSTANT_VS    * 1e3 / (1.0/self.NUMBER_OF_POLE_PAIRS/2/math.pi*60)
        this.AMPL_INVARIANT_BACK_EMF_CONSTANT_MVrms_PER_RPM = this.AMPL_INVARIANT_BACK_EMF_CONSTANT_VrmsS * 1e3 / (1.0/self.NUMBER_OF_POLE_PAIRS/2/math.pi*60)

        # E (from aplitude-invariant Clarke transform)
        this.BACK_EMF_V    = 1e-3 * this.AMPL_INVARIANT_BACK_EMF_CONSTANT_MV_PER_RPM * self.RATED_SPEED_RPM
        this.BACK_EMF_Vrms = 1e-3 * this.AMPL_INVARIANT_BACK_EMF_CONSTANT_MV_PER_RPM * self.RATED_SPEED_RPM / 1.414

        # Linear Machine Quantities
        if False:
            # Below is valid for linear motor

            # 磁铁同电极距离（N极到N极）
            this.ELECTRICAL_NORTH_NORTH_PITCH_MM = 30
            # 一台直线电机的跨度包含有 npp 个同电极距离
            this.MECHANICAL_ONE_CYCLE_TRAVLE_MM = this.ELECTRICAL_NORTH_NORTH_PITCH_MM * self.NUMBER_OF_POLE_PAIRS
            # 如果把这台直线电机掰弯成一个圆，那么其半径为
            this.RADIUS_MM = this.MECHANICAL_ONE_CYCLE_TRAVLE_MM / (2*math.pi)
            # 额定的推力为
            this.RATED_FORCE = this.RATED_TORQUE / (RADIUS_MM*1e-3)
            # 推力系数为
            this.FORCE_CONSTANT_N_PER_Arms  = this.RATED_FORCE / self.RATED_CURRENT_RMS
            this.FORCE_CONSTANT_N_PER_Apeak = this.RATED_FORCE / (self.RATED_CURRENT_RMS*1.414)
            # 电机直线速度
            this.RATED_SPEED_MM_PER_SEC = self.RATED_SPEED_RPM / 60 * (this.ELECTRICAL_NORTH_NORTH_PITCH_MM*self.NUMBER_OF_POLE_PAIRS)

        print('---------\n',
        f'{this.RATED_TORQUE:.4f} Nm\n',
        f'{this.TORQUE_CONSTANT_Nm_PER_Arms:.3f} Nm/Arms\n',
        f'{this.AMPL_INVARIANT_BACK_EMF_CONSTANT_VS:g}, Vs\n',
        f'直流伺服用：{this.AMPL_INVARIANT_BACK_EMF_CONSTANT_MV_PER_RPM:.3f} mV/rpm\n',
        f'交流伺服用：{this.AMPL_INVARIANT_BACK_EMF_CONSTANT_MVrms_PER_RPM:.3f} mVrms/rpm\n',
        f'直流伺服用：{this.BACK_EMF_V:.3f} V\n',
        f'交流伺服用：{this.BACK_EMF_Vrms:.3f} Vrms\n',
        # f'{this.RATED_SPEED_MM_PER_SEC} mm/s\n',
            sep='\t')

    def convert_KE_to_mVperRPM(self, KE):
        # Vs -> mV/RPM
        return KE * 1e3 / (1.0/self.NUMBER_OF_POLE_PAIRS/2/math.pi*60)

if __name__ == '__main__':
    em = ElectricMachinery( NUMBER_OF_POLE_PAIRS = 4,
                                       RATED_CURRENT_RMS = 12.8,
                                       RATED_POWER_WATT = 400,
                                       RATED_SPEED_RPM = 3000
        )

