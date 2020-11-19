import utils.conversion


# NUMBER_OF_POLE_PAIRS = 1
# RATED_CURRENT_RMS = 0.8
# RATED_POWER_WATT = 1.32
# RATED_SPEED_RPM = 1000

# NUMBER_OF_POLE_PAIRS = 4
# RATED_CURRENT_RMS = 6.5
# RATED_POWER_WATT = 200
# RATED_SPEED_RPM = 3000

# NUMBER_OF_POLE_PAIRS = 4
# RATED_CURRENT_RMS = 12.8
# RATED_POWER_WATT = 400
# RATED_SPEED_RPM = 3000

# NUMBER_OF_POLE_PAIRS = 4
# RATED_CURRENT_RMS = 3.5
# RATED_POWER_WATT = 50
# RATED_SPEED_RPM = 3000

# NUMBER_OF_POLE_PAIRS = 1
# RATED_CURRENT_RMS = 2.4
# RATED_POWER_WATT = 43
# RATED_SPEED_RPM = 3000

# NUMBER_OF_POLE_PAIRS = 4
# RATED_CURRENT_RMS = 12.8
# RATED_POWER_WATT = 400
# RATED_SPEED_RPM = 3000


def main():
    em = utils.conversion.ElectricMachinery( NUMBER_OF_POLE_PAIRS = 4,
                                       RATED_CURRENT_RMS = 12.8,
                                       RATED_POWER_WATT = 400,
                                       RATED_SPEED_RPM = 3000
        )

if __name__ == '__main__':
    main()


