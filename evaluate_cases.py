from run_simulation import evaluate_bus_dynamics

NUMBER_OF_RUNS = 200


for i in range(NUMBER_OF_RUNS):
    # Any door, no ticket checks
    evaluate_bus_dynamics(20, 4, 4, False, False, "outputs/any_door-no_check_20_4.csv")
    evaluate_bus_dynamics(40, 8, 8, False, False, "outputs/any_door-no_check_40_8.csv")
    evaluate_bus_dynamics(
        60, 12, 12, False, False, "outputs/any_door-no_check_60_12.csv"
    )
    evaluate_bus_dynamics(
        80, 16, 16, False, False, "outputs/any_door-no_check_80_16.csv"
    )

    # Front door entry, no ticket checks
    evaluate_bus_dynamics(20, 4, 4, True, False, "outputs/front_door-no_check_20_4.csv")
    evaluate_bus_dynamics(40, 8, 8, True, False, "outputs/front_door-no_check_40_8.csv")
    evaluate_bus_dynamics(
        60, 12, 12, True, False, "outputs/front_door-no_check_60_12.csv"
    )
    evaluate_bus_dynamics(
        80, 16, 16, True, False, "outputs/front_door-no_check_80_16.csv"
    )

    # Any door, with ticket checks
    evaluate_bus_dynamics(20, 4, 4, False, True, "outputs/any_door-with_check_20_4.csv")
    evaluate_bus_dynamics(40, 8, 8, False, True, "outputs/any_door-with_check_40_8.csv")
    evaluate_bus_dynamics(
        60, 12, 12, False, True, "outputs/any_door-with_check_60_12.csv"
    )
    evaluate_bus_dynamics(
        80, 16, 16, False, True, "outputs/any_door-with_check_80_16.csv"
    )

    # Front door entry, with ticket checks
    evaluate_bus_dynamics(
        20, 4, 4, True, True, "outputs/front_door-with_check_20_4.csv"
    )
    evaluate_bus_dynamics(
        40, 8, 8, True, True, "outputs/front_door-with_check_40_8.csv"
    )
    evaluate_bus_dynamics(
        60, 12, 12, True, True, "outputs/front_door-with_check_60_12.csv"
    )
    evaluate_bus_dynamics(
        80, 16, 16, True, True, "outputs/front_door-with_check_80_16.csv"
    )

    print(f"Finished eval round {i}")
