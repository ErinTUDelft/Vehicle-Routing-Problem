\ LP format - for model browsing. Use MPS format to capture full model detail.
Minimize
  T + 0 x[0,0,0,0] + 0 x[0,0,0,1] + 0 x[0,0,1,0] + 0 x[0,0,1,1]
   + 0 x[0,0,2,0] + 0 x[0,0,2,1] + 0 x[0,0,3,0] + 0 x[0,0,3,1]
   + 0 x[1,1,0,0] + 0 x[1,1,0,1] + 0 x[1,1,1,0] + 0 x[1,1,1,1]
   + 0 x[1,1,2,0] + 0 x[1,1,2,1] + 0 x[1,1,3,0] + 0 x[1,1,3,1]
   + 0 x[2,2,0,0] + 0 x[2,2,0,1] + 0 x[2,2,1,0] + 0 x[2,2,1,1]
   + 0 x[2,2,2,0] + 0 x[2,2,2,1] + 0 x[2,2,3,0] + 0 x[2,2,3,1]
   + 0 x[3,3,0,0] + 0 x[3,3,0,1] + 0 x[3,3,1,0] + 0 x[3,3,1,1]
   + 0 x[3,3,2,0] + 0 x[3,3,2,1] + 0 x[3,3,3,0] + 0 x[3,3,3,1] + arr[0,0,1]
   + arr[0,1,1] + arr[0,2,1] + arr[0,3,1] + 1.878117436390945 Tcomp[1]
   + 1.027387593197926 Tcomp[2] + 1.670467510178402 Tcomp[3]
Subject To
 node_1_satisfy_pesticide: p[1,0,0] + p[1,0,1] + p[1,1,0] + p[1,1,1]
   + p[1,2,0] + p[1,2,1] + p[1,3,0] + p[1,3,1] >= 7.694083670016784
 node_2_satisfy_pesticide: p[2,0,0] + p[2,0,1] + p[2,1,0] + p[2,1,1]
   + p[2,2,0] + p[2,2,1] + p[2,3,0] + p[2,3,1] >= 7.095972572016474
 node_3_satisfy_pesticide: p[3,0,0] + p[3,0,1] + p[3,1,0] + p[3,1,1]
   + p[3,2,0] + p[3,2,1] + p[3,3,0] + p[3,3,1] >= 8.426097501983797
 drone_0_can_drop_at_0_in_trip_0: - 3600 x[0,1,0,0] - 3600 x[0,2,0,0]
   - 3600 x[0,3,0,0] + p[0,0,0] <= 0
 drone_0_can_drop_at_0_in_trip_1: - 3600 x[0,1,0,1] - 3600 x[0,2,0,1]
   - 3600 x[0,3,0,1] + p[0,0,1] <= 0
 drone_1_can_drop_at_0_in_trip_0: - 3600 x[0,1,1,0] - 3600 x[0,2,1,0]
   - 3600 x[0,3,1,0] + p[0,1,0] <= 0
 drone_1_can_drop_at_0_in_trip_1: - 3600 x[0,1,1,1] - 3600 x[0,2,1,1]
   - 3600 x[0,3,1,1] + p[0,1,1] <= 0
 drone_2_can_drop_at_0_in_trip_0: - 3600 x[0,1,2,0] - 3600 x[0,2,2,0]
   - 3600 x[0,3,2,0] + p[0,2,0] <= 0
 drone_2_can_drop_at_0_in_trip_1: - 3600 x[0,1,2,1] - 3600 x[0,2,2,1]
   - 3600 x[0,3,2,1] + p[0,2,1] <= 0
 drone_3_can_drop_at_0_in_trip_0: - 3600 x[0,1,3,0] - 3600 x[0,2,3,0]
   - 3600 x[0,3,3,0] + p[0,3,0] <= 0
 drone_3_can_drop_at_0_in_trip_1: - 3600 x[0,1,3,1] - 3600 x[0,2,3,1]
   - 3600 x[0,3,3,1] + p[0,3,1] <= 0
 drone_0_can_drop_at_1_in_trip_0: - 3600 x[1,0,0,0] - 3600 x[1,2,0,0]
   - 3600 x[1,3,0,0] + p[1,0,0] <= 0
 drone_0_can_drop_at_1_in_trip_1: - 3600 x[1,0,0,1] - 3600 x[1,2,0,1]
   - 3600 x[1,3,0,1] + p[1,0,1] <= 0
 drone_1_can_drop_at_1_in_trip_0: - 3600 x[1,0,1,0] - 3600 x[1,2,1,0]
   - 3600 x[1,3,1,0] + p[1,1,0] <= 0
 drone_1_can_drop_at_1_in_trip_1: - 3600 x[1,0,1,1] - 3600 x[1,2,1,1]
   - 3600 x[1,3,1,1] + p[1,1,1] <= 0
 drone_2_can_drop_at_1_in_trip_0: - 3600 x[1,0,2,0] - 3600 x[1,2,2,0]
   - 3600 x[1,3,2,0] + p[1,2,0] <= 0
 drone_2_can_drop_at_1_in_trip_1: - 3600 x[1,0,2,1] - 3600 x[1,2,2,1]
   - 3600 x[1,3,2,1] + p[1,2,1] <= 0
 drone_3_can_drop_at_1_in_trip_0: - 3600 x[1,0,3,0] - 3600 x[1,2,3,0]
   - 3600 x[1,3,3,0] + p[1,3,0] <= 0
 drone_3_can_drop_at_1_in_trip_1: - 3600 x[1,0,3,1] - 3600 x[1,2,3,1]
   - 3600 x[1,3,3,1] + p[1,3,1] <= 0
 drone_0_can_drop_at_2_in_trip_0: - 3600 x[2,0,0,0] - 3600 x[2,1,0,0]
   - 3600 x[2,3,0,0] + p[2,0,0] <= 0
 drone_0_can_drop_at_2_in_trip_1: - 3600 x[2,0,0,1] - 3600 x[2,1,0,1]
   - 3600 x[2,3,0,1] + p[2,0,1] <= 0
 drone_1_can_drop_at_2_in_trip_0: - 3600 x[2,0,1,0] - 3600 x[2,1,1,0]
   - 3600 x[2,3,1,0] + p[2,1,0] <= 0
 drone_1_can_drop_at_2_in_trip_1: - 3600 x[2,0,1,1] - 3600 x[2,1,1,1]
   - 3600 x[2,3,1,1] + p[2,1,1] <= 0
 drone_2_can_drop_at_2_in_trip_0: - 3600 x[2,0,2,0] - 3600 x[2,1,2,0]
   - 3600 x[2,3,2,0] + p[2,2,0] <= 0
 drone_2_can_drop_at_2_in_trip_1: - 3600 x[2,0,2,1] - 3600 x[2,1,2,1]
   - 3600 x[2,3,2,1] + p[2,2,1] <= 0
 drone_3_can_drop_at_2_in_trip_0: - 3600 x[2,0,3,0] - 3600 x[2,1,3,0]
   - 3600 x[2,3,3,0] + p[2,3,0] <= 0
 drone_3_can_drop_at_2_in_trip_1: - 3600 x[2,0,3,1] - 3600 x[2,1,3,1]
   - 3600 x[2,3,3,1] + p[2,3,1] <= 0
 drone_0_can_drop_at_3_in_trip_0: - 3600 x[3,0,0,0] - 3600 x[3,1,0,0]
   - 3600 x[3,2,0,0] + p[3,0,0] <= 0
 drone_0_can_drop_at_3_in_trip_1: - 3600 x[3,0,0,1] - 3600 x[3,1,0,1]
   - 3600 x[3,2,0,1] + p[3,0,1] <= 0
 drone_1_can_drop_at_3_in_trip_0: - 3600 x[3,0,1,0] - 3600 x[3,1,1,0]
   - 3600 x[3,2,1,0] + p[3,1,0] <= 0
 drone_1_can_drop_at_3_in_trip_1: - 3600 x[3,0,1,1] - 3600 x[3,1,1,1]
   - 3600 x[3,2,1,1] + p[3,1,1] <= 0
 drone_2_can_drop_at_3_in_trip_0: - 3600 x[3,0,2,0] - 3600 x[3,1,2,0]
   - 3600 x[3,2,2,0] + p[3,2,0] <= 0
 drone_2_can_drop_at_3_in_trip_1: - 3600 x[3,0,2,1] - 3600 x[3,1,2,1]
   - 3600 x[3,2,2,1] + p[3,2,1] <= 0
 drone_3_can_drop_at_3_in_trip_0: - 3600 x[3,0,3,0] - 3600 x[3,1,3,0]
   - 3600 x[3,2,3,0] + p[3,3,0] <= 0
 drone_3_can_drop_at_3_in_trip_1: - 3600 x[3,0,3,1] - 3600 x[3,1,3,1]
   - 3600 x[3,2,3,1] + p[3,3,1] <= 0
 drone_0_can_carry_amount_of_pesticide_in_trip_0: p[0,0,0] + p[1,0,0]
   + p[2,0,0] + p[3,0,0] <= 5
 drone_0_can_carry_amount_of_pesticide_in_trip_1: p[0,0,1] + p[1,0,1]
   + p[2,0,1] + p[3,0,1] <= 5
 drone_1_can_carry_amount_of_pesticide_in_trip_0: p[0,1,0] + p[1,1,0]
   + p[2,1,0] + p[3,1,0] <= 5
 drone_1_can_carry_amount_of_pesticide_in_trip_1: p[0,1,1] + p[1,1,1]
   + p[2,1,1] + p[3,1,1] <= 5
 drone_2_can_carry_amount_of_pesticide_in_trip_0: p[0,2,0] + p[1,2,0]
   + p[2,2,0] + p[3,2,0] <= 5
 drone_2_can_carry_amount_of_pesticide_in_trip_1: p[0,2,1] + p[1,2,1]
   + p[2,2,1] + p[3,2,1] <= 5
 drone_3_can_carry_amount_of_pesticide_in_trip_0: p[0,3,0] + p[1,3,0]
   + p[2,3,0] + p[3,3,0] <= 5
 drone_3_can_carry_amount_of_pesticide_in_trip_1: p[0,3,1] + p[1,3,1]
   + p[2,3,1] + p[3,3,1] <= 5
 drone_0_starts_at_origin: dep[0,0,0] = 0
 drone_1_starts_at_origin: dep[0,1,0] = 0
 drone_2_starts_at_origin: dep[0,2,0] = 0
 drone_3_starts_at_origin: dep[0,3,0] = 0
 travel_time_from0to1_drone0_trip0: 3600 x[0,1,0,0] + dep[0,0,0]
   - arr[1,0,0] <= 3574.321209331971
 travel_time_from0to1_drone0_trip1: 3600 x[0,1,0,1] + dep[0,0,1]
   - arr[1,0,1] <= 3574.321209331971
 travel_time_from0to1_drone1_trip0: 3600 x[0,1,1,0] + dep[0,1,0]
   - arr[1,1,0] <= 3574.321209331971
 travel_time_from0to1_drone1_trip1: 3600 x[0,1,1,1] + dep[0,1,1]
   - arr[1,1,1] <= 3574.321209331971
 travel_time_from0to1_drone2_trip0: 3600 x[0,1,2,0] + dep[0,2,0]
   - arr[1,2,0] <= 3574.321209331971
 travel_time_from0to1_drone2_trip1: 3600 x[0,1,2,1] + dep[0,2,1]
   - arr[1,2,1] <= 3574.321209331971
 travel_time_from0to1_drone3_trip0: 3600 x[0,1,3,0] + dep[0,3,0]
   - arr[1,3,0] <= 3574.321209331971
 travel_time_from0to1_drone3_trip1: 3600 x[0,1,3,1] + dep[0,3,1]
   - arr[1,3,1] <= 3574.321209331971
 travel_time_from0to2_drone0_trip0: 3600 x[0,2,0,0] + dep[0,0,0]
   - arr[2,0,0] <= 3565.10207710739
 travel_time_from0to2_drone0_trip1: 3600 x[0,2,0,1] + dep[0,0,1]
   - arr[2,0,1] <= 3565.10207710739
 travel_time_from0to2_drone1_trip0: 3600 x[0,2,1,0] + dep[0,1,0]
   - arr[2,1,0] <= 3565.10207710739
 travel_time_from0to2_drone1_trip1: 3600 x[0,2,1,1] + dep[0,1,1]
   - arr[2,1,1] <= 3565.10207710739
 travel_time_from0to2_drone2_trip0: 3600 x[0,2,2,0] + dep[0,2,0]
   - arr[2,2,0] <= 3565.10207710739
 travel_time_from0to2_drone2_trip1: 3600 x[0,2,2,1] + dep[0,2,1]
   - arr[2,2,1] <= 3565.10207710739
 travel_time_from0to2_drone3_trip0: 3600 x[0,2,3,0] + dep[0,3,0]
   - arr[2,3,0] <= 3565.10207710739
 travel_time_from0to2_drone3_trip1: 3600 x[0,2,3,1] + dep[0,3,1]
   - arr[2,3,1] <= 3565.10207710739
 travel_time_from0to3_drone0_trip0: 3600 x[0,3,0,0] + dep[0,0,0]
   - arr[3,0,0] <= 3580.873769332347
 travel_time_from0to3_drone0_trip1: 3600 x[0,3,0,1] + dep[0,0,1]
   - arr[3,0,1] <= 3580.873769332347
 travel_time_from0to3_drone1_trip0: 3600 x[0,3,1,0] + dep[0,1,0]
   - arr[3,1,0] <= 3580.873769332347
 travel_time_from0to3_drone1_trip1: 3600 x[0,3,1,1] + dep[0,1,1]
   - arr[3,1,1] <= 3580.873769332347
 travel_time_from0to3_drone2_trip0: 3600 x[0,3,2,0] + dep[0,2,0]
   - arr[3,2,0] <= 3580.873769332347
 travel_time_from0to3_drone2_trip1: 3600 x[0,3,2,1] + dep[0,2,1]
   - arr[3,2,1] <= 3580.873769332347
 travel_time_from0to3_drone3_trip0: 3600 x[0,3,3,0] + dep[0,3,0]
   - arr[3,3,0] <= 3580.873769332347
 travel_time_from0to3_drone3_trip1: 3600 x[0,3,3,1] + dep[0,3,1]
   - arr[3,3,1] <= 3580.873769332347
 travel_time_from1to0_drone0_trip0: 3600 x[1,0,0,0] - arr[0,0,0]
   + dep[1,0,0] <= 3574.321209331971
 travel_time_from1to0_drone0_trip1: 3600 x[1,0,0,1] - arr[0,0,1]
   + dep[1,0,1] <= 3574.321209331971
 travel_time_from1to0_drone1_trip0: 3600 x[1,0,1,0] - arr[0,1,0]
   + dep[1,1,0] <= 3574.321209331971
 travel_time_from1to0_drone1_trip1: 3600 x[1,0,1,1] - arr[0,1,1]
   + dep[1,1,1] <= 3574.321209331971
 travel_time_from1to0_drone2_trip0: 3600 x[1,0,2,0] - arr[0,2,0]
   + dep[1,2,0] <= 3574.321209331971
 travel_time_from1to0_drone2_trip1: 3600 x[1,0,2,1] - arr[0,2,1]
   + dep[1,2,1] <= 3574.321209331971
 travel_time_from1to0_drone3_trip0: 3600 x[1,0,3,0] - arr[0,3,0]
   + dep[1,3,0] <= 3574.321209331971
 travel_time_from1to0_drone3_trip1: 3600 x[1,0,3,1] - arr[0,3,1]
   + dep[1,3,1] <= 3574.321209331971
 travel_time_from1to2_drone0_trip0: 3600 x[1,2,0,0] + dep[1,0,0]
   - arr[2,0,0] <= 3539.474300394379
 travel_time_from1to2_drone0_trip1: 3600 x[1,2,0,1] + dep[1,0,1]
   - arr[2,0,1] <= 3539.474300394379
 travel_time_from1to2_drone1_trip0: 3600 x[1,2,1,0] + dep[1,1,0]
   - arr[2,1,0] <= 3539.474300394379
 travel_time_from1to2_drone1_trip1: 3600 x[1,2,1,1] + dep[1,1,1]
   - arr[2,1,1] <= 3539.474300394379
 travel_time_from1to2_drone2_trip0: 3600 x[1,2,2,0] + dep[1,2,0]
   - arr[2,2,0] <= 3539.474300394379
 travel_time_from1to2_drone2_trip1: 3600 x[1,2,2,1] + dep[1,2,1]
   - arr[2,2,1] <= 3539.474300394379
 travel_time_from1to2_drone3_trip0: 3600 x[1,2,3,0] + dep[1,3,0]
   - arr[2,3,0] <= 3539.474300394379
 travel_time_from1to2_drone3_trip1: 3600 x[1,2,3,1] + dep[1,3,1]
   - arr[2,3,1] <= 3539.474300394379
 travel_time_from1to3_drone0_trip0: 3600 x[1,3,0,0] + dep[1,0,0]
   - arr[3,0,0] <= 3559.274061795796
 travel_time_from1to3_drone0_trip1: 3600 x[1,3,0,1] + dep[1,0,1]
   - arr[3,0,1] <= 3559.274061795796
 travel_time_from1to3_drone1_trip0: 3600 x[1,3,1,0] + dep[1,1,0]
   - arr[3,1,0] <= 3559.274061795796
 travel_time_from1to3_drone1_trip1: 3600 x[1,3,1,1] + dep[1,1,1]
   - arr[3,1,1] <= 3559.274061795796
 travel_time_from1to3_drone2_trip0: 3600 x[1,3,2,0] + dep[1,2,0]
   - arr[3,2,0] <= 3559.274061795796
 travel_time_from1to3_drone2_trip1: 3600 x[1,3,2,1] + dep[1,2,1]
   - arr[3,2,1] <= 3559.274061795796
 travel_time_from1to3_drone3_trip0: 3600 x[1,3,3,0] + dep[1,3,0]
   - arr[3,3,0] <= 3559.274061795796
 travel_time_from1to3_drone3_trip1: 3600 x[1,3,3,1] + dep[1,3,1]
   - arr[3,3,1] <= 3559.274061795796
 travel_time_from2to0_drone0_trip0: 3600 x[2,0,0,0] - arr[0,0,0]
   + dep[2,0,0] <= 3565.10207710739
 travel_time_from2to0_drone0_trip1: 3600 x[2,0,0,1] - arr[0,0,1]
   + dep[2,0,1] <= 3565.10207710739
 travel_time_from2to0_drone1_trip0: 3600 x[2,0,1,0] - arr[0,1,0]
   + dep[2,1,0] <= 3565.10207710739
 travel_time_from2to0_drone1_trip1: 3600 x[2,0,1,1] - arr[0,1,1]
   + dep[2,1,1] <= 3565.10207710739
 travel_time_from2to0_drone2_trip0: 3600 x[2,0,2,0] - arr[0,2,0]
   + dep[2,2,0] <= 3565.10207710739
 travel_time_from2to0_drone2_trip1: 3600 x[2,0,2,1] - arr[0,2,1]
   + dep[2,2,1] <= 3565.10207710739
 travel_time_from2to0_drone3_trip0: 3600 x[2,0,3,0] - arr[0,3,0]
   + dep[2,3,0] <= 3565.10207710739
 travel_time_from2to0_drone3_trip1: 3600 x[2,0,3,1] - arr[0,3,1]
   + dep[2,3,1] <= 3565.10207710739
 travel_time_from2to1_drone0_trip0: 3600 x[2,1,0,0] - arr[1,0,0]
   + dep[2,0,0] <= 3539.474300394379
 travel_time_from2to1_drone0_trip1: 3600 x[2,1,0,1] - arr[1,0,1]
   + dep[2,0,1] <= 3539.474300394379
 travel_time_from2to1_drone1_trip0: 3600 x[2,1,1,0] - arr[1,1,0]
   + dep[2,1,0] <= 3539.474300394379
 travel_time_from2to1_drone1_trip1: 3600 x[2,1,1,1] - arr[1,1,1]
   + dep[2,1,1] <= 3539.474300394379
 travel_time_from2to1_drone2_trip0: 3600 x[2,1,2,0] - arr[1,2,0]
   + dep[2,2,0] <= 3539.474300394379
 travel_time_from2to1_drone2_trip1: 3600 x[2,1,2,1] - arr[1,2,1]
   + dep[2,2,1] <= 3539.474300394379
 travel_time_from2to1_drone3_trip0: 3600 x[2,1,3,0] - arr[1,3,0]
   + dep[2,3,0] <= 3539.474300394379
 travel_time_from2to1_drone3_trip1: 3600 x[2,1,3,1] - arr[1,3,1]
   + dep[2,3,1] <= 3539.474300394379
 travel_time_from2to3_drone0_trip0: 3600 x[2,3,0,0] + dep[2,0,0]
   - arr[3,0,0] <= 3571.5306581344
 travel_time_from2to3_drone0_trip1: 3600 x[2,3,0,1] + dep[2,0,1]
   - arr[3,0,1] <= 3571.5306581344
 travel_time_from2to3_drone1_trip0: 3600 x[2,3,1,0] + dep[2,1,0]
   - arr[3,1,0] <= 3571.5306581344
 travel_time_from2to3_drone1_trip1: 3600 x[2,3,1,1] + dep[2,1,1]
   - arr[3,1,1] <= 3571.5306581344
 travel_time_from2to3_drone2_trip0: 3600 x[2,3,2,0] + dep[2,2,0]
   - arr[3,2,0] <= 3571.5306581344
 travel_time_from2to3_drone2_trip1: 3600 x[2,3,2,1] + dep[2,2,1]
   - arr[3,2,1] <= 3571.5306581344
 travel_time_from2to3_drone3_trip0: 3600 x[2,3,3,0] + dep[2,3,0]
   - arr[3,3,0] <= 3571.5306581344
 travel_time_from2to3_drone3_trip1: 3600 x[2,3,3,1] + dep[2,3,1]
   - arr[3,3,1] <= 3571.5306581344
 travel_time_from3to0_drone0_trip0: 3600 x[3,0,0,0] - arr[0,0,0]
   + dep[3,0,0] <= 3580.873769332347
 travel_time_from3to0_drone0_trip1: 3600 x[3,0,0,1] - arr[0,0,1]
   + dep[3,0,1] <= 3580.873769332347
 travel_time_from3to0_drone1_trip0: 3600 x[3,0,1,0] - arr[0,1,0]
   + dep[3,1,0] <= 3580.873769332347
 travel_time_from3to0_drone1_trip1: 3600 x[3,0,1,1] - arr[0,1,1]
   + dep[3,1,1] <= 3580.873769332347
 travel_time_from3to0_drone2_trip0: 3600 x[3,0,2,0] - arr[0,2,0]
   + dep[3,2,0] <= 3580.873769332347
 travel_time_from3to0_drone2_trip1: 3600 x[3,0,2,1] - arr[0,2,1]
   + dep[3,2,1] <= 3580.873769332347
 travel_time_from3to0_drone3_trip0: 3600 x[3,0,3,0] - arr[0,3,0]
   + dep[3,3,0] <= 3580.873769332347
 travel_time_from3to0_drone3_trip1: 3600 x[3,0,3,1] - arr[0,3,1]
   + dep[3,3,1] <= 3580.873769332347
 travel_time_from3to1_drone0_trip0: 3600 x[3,1,0,0] - arr[1,0,0]
   + dep[3,0,0] <= 3559.274061795796
 travel_time_from3to1_drone0_trip1: 3600 x[3,1,0,1] - arr[1,0,1]
   + dep[3,0,1] <= 3559.274061795796
 travel_time_from3to1_drone1_trip0: 3600 x[3,1,1,0] - arr[1,1,0]
   + dep[3,1,0] <= 3559.274061795796
 travel_time_from3to1_drone1_trip1: 3600 x[3,1,1,1] - arr[1,1,1]
   + dep[3,1,1] <= 3559.274061795796
 travel_time_from3to1_drone2_trip0: 3600 x[3,1,2,0] - arr[1,2,0]
   + dep[3,2,0] <= 3559.274061795796
 travel_time_from3to1_drone2_trip1: 3600 x[3,1,2,1] - arr[1,2,1]
   + dep[3,2,1] <= 3559.274061795796
 travel_time_from3to1_drone3_trip0: 3600 x[3,1,3,0] - arr[1,3,0]
   + dep[3,3,0] <= 3559.274061795796
 travel_time_from3to1_drone3_trip1: 3600 x[3,1,3,1] - arr[1,3,1]
   + dep[3,3,1] <= 3559.274061795796
 travel_time_from3to2_drone0_trip0: 3600 x[3,2,0,0] - arr[2,0,0]
   + dep[3,0,0] <= 3571.5306581344
 travel_time_from3to2_drone0_trip1: 3600 x[3,2,0,1] - arr[2,0,1]
   + dep[3,0,1] <= 3571.5306581344
 travel_time_from3to2_drone1_trip0: 3600 x[3,2,1,0] - arr[2,1,0]
   + dep[3,1,0] <= 3571.5306581344
 travel_time_from3to2_drone1_trip1: 3600 x[3,2,1,1] - arr[2,1,1]
   + dep[3,1,1] <= 3571.5306581344
 travel_time_from3to2_drone2_trip0: 3600 x[3,2,2,0] - arr[2,2,0]
   + dep[3,2,0] <= 3571.5306581344
 travel_time_from3to2_drone2_trip1: 3600 x[3,2,2,1] - arr[2,2,1]
   + dep[3,2,1] <= 3571.5306581344
 travel_time_from3to2_drone3_trip0: 3600 x[3,2,3,0] - arr[2,3,0]
   + dep[3,3,0] <= 3571.5306581344
 travel_time_from3to2_drone3_trip1: 3600 x[3,2,3,1] - arr[2,3,1]
   + dep[3,3,1] <= 3571.5306581344
 drop_time_node1_drone0_trip0: 10 p[1,0,0] + arr[1,0,0] - dep[1,0,0] = 0
 drop_time_node1_drone0_trip1: 10 p[1,0,1] + arr[1,0,1] - dep[1,0,1] = 0
 drop_time_node1_drone1_trip0: 10 p[1,1,0] + arr[1,1,0] - dep[1,1,0] = 0
 drop_time_node1_drone1_trip1: 10 p[1,1,1] + arr[1,1,1] - dep[1,1,1] = 0
 drop_time_node1_drone2_trip0: 10 p[1,2,0] + arr[1,2,0] - dep[1,2,0] = 0
 drop_time_node1_drone2_trip1: 10 p[1,2,1] + arr[1,2,1] - dep[1,2,1] = 0
 drop_time_node1_drone3_trip0: 10 p[1,3,0] + arr[1,3,0] - dep[1,3,0] = 0
 drop_time_node1_drone3_trip1: 10 p[1,3,1] + arr[1,3,1] - dep[1,3,1] = 0
 drop_time_node2_drone0_trip0: 10 p[2,0,0] + arr[2,0,0] - dep[2,0,0] = 0
 drop_time_node2_drone0_trip1: 10 p[2,0,1] + arr[2,0,1] - dep[2,0,1] = 0
 drop_time_node2_drone1_trip0: 10 p[2,1,0] + arr[2,1,0] - dep[2,1,0] = 0
 drop_time_node2_drone1_trip1: 10 p[2,1,1] + arr[2,1,1] - dep[2,1,1] = 0
 drop_time_node2_drone2_trip0: 10 p[2,2,0] + arr[2,2,0] - dep[2,2,0] = 0
 drop_time_node2_drone2_trip1: 10 p[2,2,1] + arr[2,2,1] - dep[2,2,1] = 0
 drop_time_node2_drone3_trip0: 10 p[2,3,0] + arr[2,3,0] - dep[2,3,0] = 0
 drop_time_node2_drone3_trip1: 10 p[2,3,1] + arr[2,3,1] - dep[2,3,1] = 0
 drop_time_node3_drone0_trip0: 10 p[3,0,0] + arr[3,0,0] - dep[3,0,0] = 0
 drop_time_node3_drone0_trip1: 10 p[3,0,1] + arr[3,0,1] - dep[3,0,1] = 0
 drop_time_node3_drone1_trip0: 10 p[3,1,0] + arr[3,1,0] - dep[3,1,0] = 0
 drop_time_node3_drone1_trip1: 10 p[3,1,1] + arr[3,1,1] - dep[3,1,1] = 0
 drop_time_node3_drone2_trip0: 10 p[3,2,0] + arr[3,2,0] - dep[3,2,0] = 0
 drop_time_node3_drone2_trip1: 10 p[3,2,1] + arr[3,2,1] - dep[3,2,1] = 0
 drop_time_node3_drone3_trip0: 10 p[3,3,0] + arr[3,3,0] - dep[3,3,0] = 0
 drop_time_node3_drone3_trip1: 10 p[3,3,1] + arr[3,3,1] - dep[3,3,1] = 0
 refill_time_drone0_trip1: arr[0,0,0] - dep[0,0,1] + 30 a[0,1] <= 0
 refill_time_drone1_trip1: arr[0,1,0] - dep[0,1,1] + 30 a[1,1] <= 0
 refill_time_drone2_trip1: arr[0,2,0] - dep[0,2,1] + 30 a[2,1] <= 0
 refill_time_drone3_trip1: arr[0,3,0] - dep[0,3,1] + 30 a[3,1] <= 0
 for_drone_0_for_trip_1_to_happen_trip_0_must_happen: a[0,0] - a[0,1] >= 0
 for_drone_1_for_trip_1_to_happen_trip_0_must_happen: a[1,0] - a[1,1] >= 0
 for_drone_2_for_trip_1_to_happen_trip_0_must_happen: a[2,0] - a[2,1] >= 0
 for_drone_3_for_trip_1_to_happen_trip_0_must_happen: a[3,0] - a[3,1] >= 0
 drone_0_must_arrive_at_origin_after_it_leaves_it_in_trip_0: - arr[0,0,0]
   + dep[0,0,0] <= 0
 drone_0_must_arrive_at_origin_after_it_leaves_it_in_trip_1: - arr[0,0,1]
   + dep[0,0,1] <= 0
 drone_1_must_arrive_at_origin_after_it_leaves_it_in_trip_0: - arr[0,1,0]
   + dep[0,1,0] <= 0
 drone_1_must_arrive_at_origin_after_it_leaves_it_in_trip_1: - arr[0,1,1]
   + dep[0,1,1] <= 0
 drone_2_must_arrive_at_origin_after_it_leaves_it_in_trip_0: - arr[0,2,0]
   + dep[0,2,0] <= 0
 drone_2_must_arrive_at_origin_after_it_leaves_it_in_trip_1: - arr[0,2,1]
   + dep[0,2,1] <= 0
 drone_3_must_arrive_at_origin_after_it_leaves_it_in_trip_0: - arr[0,3,0]
   + dep[0,3,0] <= 0
 drone_3_must_arrive_at_origin_after_it_leaves_it_in_trip_1: - arr[0,3,1]
   + dep[0,3,1] <= 0
 in_trip_0_drone_0_must_leave_0_if_it_travels_there: x[0,1,0,0]
   + x[0,2,0,0] + x[0,3,0,0] - x[1,0,0,0] - x[2,0,0,0] - x[3,0,0,0] = 0
 in_trip_0_drone_0_must_leave_1_if_it_travels_there: - x[0,1,0,0]
   + x[1,0,0,0] + x[1,2,0,0] + x[1,3,0,0] - x[2,1,0,0] - x[3,1,0,0] = 0
 in_trip_0_drone_0_must_leave_2_if_it_travels_there: - x[0,2,0,0]
   - x[1,2,0,0] + x[2,0,0,0] + x[2,1,0,0] + x[2,3,0,0] - x[3,2,0,0] = 0
 in_trip_0_drone_0_must_leave_3_if_it_travels_there: - x[0,3,0,0]
   - x[1,3,0,0] - x[2,3,0,0] + x[3,0,0,0] + x[3,1,0,0] + x[3,2,0,0] = 0
 in_trip_1_drone_0_must_leave_0_if_it_travels_there: x[0,1,0,1]
   + x[0,2,0,1] + x[0,3,0,1] - x[1,0,0,1] - x[2,0,0,1] - x[3,0,0,1] = 0
 in_trip_1_drone_0_must_leave_1_if_it_travels_there: - x[0,1,0,1]
   + x[1,0,0,1] + x[1,2,0,1] + x[1,3,0,1] - x[2,1,0,1] - x[3,1,0,1] = 0
 in_trip_1_drone_0_must_leave_2_if_it_travels_there: - x[0,2,0,1]
   - x[1,2,0,1] + x[2,0,0,1] + x[2,1,0,1] + x[2,3,0,1] - x[3,2,0,1] = 0
 in_trip_1_drone_0_must_leave_3_if_it_travels_there: - x[0,3,0,1]
   - x[1,3,0,1] - x[2,3,0,1] + x[3,0,0,1] + x[3,1,0,1] + x[3,2,0,1] = 0
 in_trip_0_drone_1_must_leave_0_if_it_travels_there: x[0,1,1,0]
   + x[0,2,1,0] + x[0,3,1,0] - x[1,0,1,0] - x[2,0,1,0] - x[3,0,1,0] = 0
 in_trip_0_drone_1_must_leave_1_if_it_travels_there: - x[0,1,1,0]
   + x[1,0,1,0] + x[1,2,1,0] + x[1,3,1,0] - x[2,1,1,0] - x[3,1,1,0] = 0
 in_trip_0_drone_1_must_leave_2_if_it_travels_there: - x[0,2,1,0]
   - x[1,2,1,0] + x[2,0,1,0] + x[2,1,1,0] + x[2,3,1,0] - x[3,2,1,0] = 0
 in_trip_0_drone_1_must_leave_3_if_it_travels_there: - x[0,3,1,0]
   - x[1,3,1,0] - x[2,3,1,0] + x[3,0,1,0] + x[3,1,1,0] + x[3,2,1,0] = 0
 in_trip_1_drone_1_must_leave_0_if_it_travels_there: x[0,1,1,1]
   + x[0,2,1,1] + x[0,3,1,1] - x[1,0,1,1] - x[2,0,1,1] - x[3,0,1,1] = 0
 in_trip_1_drone_1_must_leave_1_if_it_travels_there: - x[0,1,1,1]
   + x[1,0,1,1] + x[1,2,1,1] + x[1,3,1,1] - x[2,1,1,1] - x[3,1,1,1] = 0
 in_trip_1_drone_1_must_leave_2_if_it_travels_there: - x[0,2,1,1]
   - x[1,2,1,1] + x[2,0,1,1] + x[2,1,1,1] + x[2,3,1,1] - x[3,2,1,1] = 0
 in_trip_1_drone_1_must_leave_3_if_it_travels_there: - x[0,3,1,1]
   - x[1,3,1,1] - x[2,3,1,1] + x[3,0,1,1] + x[3,1,1,1] + x[3,2,1,1] = 0
 in_trip_0_drone_2_must_leave_0_if_it_travels_there: x[0,1,2,0]
   + x[0,2,2,0] + x[0,3,2,0] - x[1,0,2,0] - x[2,0,2,0] - x[3,0,2,0] = 0
 in_trip_0_drone_2_must_leave_1_if_it_travels_there: - x[0,1,2,0]
   + x[1,0,2,0] + x[1,2,2,0] + x[1,3,2,0] - x[2,1,2,0] - x[3,1,2,0] = 0
 in_trip_0_drone_2_must_leave_2_if_it_travels_there: - x[0,2,2,0]
   - x[1,2,2,0] + x[2,0,2,0] + x[2,1,2,0] + x[2,3,2,0] - x[3,2,2,0] = 0
 in_trip_0_drone_2_must_leave_3_if_it_travels_there: - x[0,3,2,0]
   - x[1,3,2,0] - x[2,3,2,0] + x[3,0,2,0] + x[3,1,2,0] + x[3,2,2,0] = 0
 in_trip_1_drone_2_must_leave_0_if_it_travels_there: x[0,1,2,1]
   + x[0,2,2,1] + x[0,3,2,1] - x[1,0,2,1] - x[2,0,2,1] - x[3,0,2,1] = 0
 in_trip_1_drone_2_must_leave_1_if_it_travels_there: - x[0,1,2,1]
   + x[1,0,2,1] + x[1,2,2,1] + x[1,3,2,1] - x[2,1,2,1] - x[3,1,2,1] = 0
 in_trip_1_drone_2_must_leave_2_if_it_travels_there: - x[0,2,2,1]
   - x[1,2,2,1] + x[2,0,2,1] + x[2,1,2,1] + x[2,3,2,1] - x[3,2,2,1] = 0
 in_trip_1_drone_2_must_leave_3_if_it_travels_there: - x[0,3,2,1]
   - x[1,3,2,1] - x[2,3,2,1] + x[3,0,2,1] + x[3,1,2,1] + x[3,2,2,1] = 0
 in_trip_0_drone_3_must_leave_0_if_it_travels_there: x[0,1,3,0]
   + x[0,2,3,0] + x[0,3,3,0] - x[1,0,3,0] - x[2,0,3,0] - x[3,0,3,0] = 0
 in_trip_0_drone_3_must_leave_1_if_it_travels_there: - x[0,1,3,0]
   + x[1,0,3,0] + x[1,2,3,0] + x[1,3,3,0] - x[2,1,3,0] - x[3,1,3,0] = 0
 in_trip_0_drone_3_must_leave_2_if_it_travels_there: - x[0,2,3,0]
   - x[1,2,3,0] + x[2,0,3,0] + x[2,1,3,0] + x[2,3,3,0] - x[3,2,3,0] = 0
 in_trip_0_drone_3_must_leave_3_if_it_travels_there: - x[0,3,3,0]
   - x[1,3,3,0] - x[2,3,3,0] + x[3,0,3,0] + x[3,1,3,0] + x[3,2,3,0] = 0
 in_trip_1_drone_3_must_leave_0_if_it_travels_there: x[0,1,3,1]
   + x[0,2,3,1] + x[0,3,3,1] - x[1,0,3,1] - x[2,0,3,1] - x[3,0,3,1] = 0
 in_trip_1_drone_3_must_leave_1_if_it_travels_there: - x[0,1,3,1]
   + x[1,0,3,1] + x[1,2,3,1] + x[1,3,3,1] - x[2,1,3,1] - x[3,1,3,1] = 0
 in_trip_1_drone_3_must_leave_2_if_it_travels_there: - x[0,2,3,1]
   - x[1,2,3,1] + x[2,0,3,1] + x[2,1,3,1] + x[2,3,3,1] - x[3,2,3,1] = 0
 in_trip_1_drone_3_must_leave_3_if_it_travels_there: - x[0,3,3,1]
   - x[1,3,3,1] - x[2,3,3,1] + x[3,0,3,1] + x[3,1,3,1] + x[3,2,3,1] = 0
 drone_0_must_leave_depot_in_trip0: x[0,1,0,0] + x[0,2,0,0] + x[0,3,0,0]
   - a[0,0] = 0
 drone_0_must_leave_depot_in_trip1: x[0,1,0,1] + x[0,2,0,1] + x[0,3,0,1]
   - a[0,1] = 0
 drone_1_must_leave_depot_in_trip0: x[0,1,1,0] + x[0,2,1,0] + x[0,3,1,0]
   - a[1,0] = 0
 drone_1_must_leave_depot_in_trip1: x[0,1,1,1] + x[0,2,1,1] + x[0,3,1,1]
   - a[1,1] = 0
 drone_2_must_leave_depot_in_trip0: x[0,1,2,0] + x[0,2,2,0] + x[0,3,2,0]
   - a[2,0] = 0
 drone_2_must_leave_depot_in_trip1: x[0,1,2,1] + x[0,2,2,1] + x[0,3,2,1]
   - a[2,1] = 0
 drone_3_must_leave_depot_in_trip0: x[0,1,3,0] + x[0,2,3,0] + x[0,3,3,0]
   - a[3,0] = 0
 drone_3_must_leave_depot_in_trip1: x[0,1,3,1] + x[0,2,3,1] + x[0,3,3,1]
   - a[3,1] = 0
 drone_0_must_return_to_depot_in_trip0: x[1,0,0,0] + x[2,0,0,0]
   + x[3,0,0,0] - a[0,0] = 0
 drone_0_must_return_to_depot_in_trip1: x[1,0,0,1] + x[2,0,0,1]
   + x[3,0,0,1] - a[0,1] = 0
 drone_1_must_return_to_depot_in_trip0: x[1,0,1,0] + x[2,0,1,0]
   + x[3,0,1,0] - a[1,0] = 0
 drone_1_must_return_to_depot_in_trip1: x[1,0,1,1] + x[2,0,1,1]
   + x[3,0,1,1] - a[1,1] = 0
 drone_2_must_return_to_depot_in_trip0: x[1,0,2,0] + x[2,0,2,0]
   + x[3,0,2,0] - a[2,0] = 0
 drone_2_must_return_to_depot_in_trip1: x[1,0,2,1] + x[2,0,2,1]
   + x[3,0,2,1] - a[2,1] = 0
 drone_3_must_return_to_depot_in_trip0: x[1,0,3,0] + x[2,0,3,0]
   + x[3,0,3,0] - a[3,0] = 0
 drone_3_must_return_to_depot_in_trip1: x[1,0,3,1] + x[2,0,3,1]
   + x[3,0,3,1] - a[3,1] = 0
 Total_time_greater_than_final_time_of_drone0: T - arr[0,0,1] >= 0
 Total_time_greater_than_final_time_of_drone1: T - arr[0,1,1] >= 0
 Total_time_greater_than_final_time_of_drone2: T - arr[0,2,1] >= 0
 Total_time_greater_than_final_time_of_drone3: T - arr[0,3,1] >= 0
 Completion_time_node1: - dep[1,0,0] + Tcomp[1] >= 0
 Completion_time_node1: - dep[1,1,0] + Tcomp[1] >= 0
 Completion_time_node1: - dep[1,2,0] + Tcomp[1] >= 0
 Completion_time_node1: - dep[1,3,0] + Tcomp[1] >= 0
 Completion_time_node1: - dep[1,0,1] + Tcomp[1] >= 0
 Completion_time_node1: - dep[1,1,1] + Tcomp[1] >= 0
 Completion_time_node1: - dep[1,2,1] + Tcomp[1] >= 0
 Completion_time_node1: - dep[1,3,1] + Tcomp[1] >= 0
 Completion_time_node2: - dep[2,0,0] + Tcomp[2] >= 0
 Completion_time_node2: - dep[2,1,0] + Tcomp[2] >= 0
 Completion_time_node2: - dep[2,2,0] + Tcomp[2] >= 0
 Completion_time_node2: - dep[2,3,0] + Tcomp[2] >= 0
 Completion_time_node2: - dep[2,0,1] + Tcomp[2] >= 0
 Completion_time_node2: - dep[2,1,1] + Tcomp[2] >= 0
 Completion_time_node2: - dep[2,2,1] + Tcomp[2] >= 0
 Completion_time_node2: - dep[2,3,1] + Tcomp[2] >= 0
 Completion_time_node3: - dep[3,0,0] + Tcomp[3] >= 0
 Completion_time_node3: - dep[3,1,0] + Tcomp[3] >= 0
 Completion_time_node3: - dep[3,2,0] + Tcomp[3] >= 0
 Completion_time_node3: - dep[3,3,0] + Tcomp[3] >= 0
 Completion_time_node3: - dep[3,0,1] + Tcomp[3] >= 0
 Completion_time_node3: - dep[3,1,1] + Tcomp[3] >= 0
 Completion_time_node3: - dep[3,2,1] + Tcomp[3] >= 0
 Completion_time_node3: - dep[3,3,1] + Tcomp[3] >= 0
Bounds
 p[0,0,0] <= 5
 p[0,0,1] <= 5
 p[0,1,0] <= 5
 p[0,1,1] <= 5
 p[0,2,0] <= 5
 p[0,2,1] <= 5
 p[0,3,0] <= 5
 p[0,3,1] <= 5
 p[1,0,0] <= 5
 p[1,0,1] <= 5
 p[1,1,0] <= 5
 p[1,1,1] <= 5
 p[1,2,0] <= 5
 p[1,2,1] <= 5
 p[1,3,0] <= 5
 p[1,3,1] <= 5
 p[2,0,0] <= 5
 p[2,0,1] <= 5
 p[2,1,0] <= 5
 p[2,1,1] <= 5
 p[2,2,0] <= 5
 p[2,2,1] <= 5
 p[2,3,0] <= 5
 p[2,3,1] <= 5
 p[3,0,0] <= 5
 p[3,0,1] <= 5
 p[3,1,0] <= 5
 p[3,1,1] <= 5
 p[3,2,0] <= 5
 p[3,2,1] <= 5
 p[3,3,0] <= 5
 p[3,3,1] <= 5
Binaries
 x[0,0,0,0] x[0,0,0,1] x[0,0,1,0] x[0,0,1,1] x[0,0,2,0] x[0,0,2,1]
 x[0,0,3,0] x[0,0,3,1] x[0,1,0,0] x[0,1,0,1] x[0,1,1,0] x[0,1,1,1]
 x[0,1,2,0] x[0,1,2,1] x[0,1,3,0] x[0,1,3,1] x[0,2,0,0] x[0,2,0,1]
 x[0,2,1,0] x[0,2,1,1] x[0,2,2,0] x[0,2,2,1] x[0,2,3,0] x[0,2,3,1]
 x[0,3,0,0] x[0,3,0,1] x[0,3,1,0] x[0,3,1,1] x[0,3,2,0] x[0,3,2,1]
 x[0,3,3,0] x[0,3,3,1] x[1,0,0,0] x[1,0,0,1] x[1,0,1,0] x[1,0,1,1]
 x[1,0,2,0] x[1,0,2,1] x[1,0,3,0] x[1,0,3,1] x[1,1,0,0] x[1,1,0,1]
 x[1,1,1,0] x[1,1,1,1] x[1,1,2,0] x[1,1,2,1] x[1,1,3,0] x[1,1,3,1]
 x[1,2,0,0] x[1,2,0,1] x[1,2,1,0] x[1,2,1,1] x[1,2,2,0] x[1,2,2,1]
 x[1,2,3,0] x[1,2,3,1] x[1,3,0,0] x[1,3,0,1] x[1,3,1,0] x[1,3,1,1]
 x[1,3,2,0] x[1,3,2,1] x[1,3,3,0] x[1,3,3,1] x[2,0,0,0] x[2,0,0,1]
 x[2,0,1,0] x[2,0,1,1] x[2,0,2,0] x[2,0,2,1] x[2,0,3,0] x[2,0,3,1]
 x[2,1,0,0] x[2,1,0,1] x[2,1,1,0] x[2,1,1,1] x[2,1,2,0] x[2,1,2,1]
 x[2,1,3,0] x[2,1,3,1] x[2,2,0,0] x[2,2,0,1] x[2,2,1,0] x[2,2,1,1]
 x[2,2,2,0] x[2,2,2,1] x[2,2,3,0] x[2,2,3,1] x[2,3,0,0] x[2,3,0,1]
 x[2,3,1,0] x[2,3,1,1] x[2,3,2,0] x[2,3,2,1] x[2,3,3,0] x[2,3,3,1]
 x[3,0,0,0] x[3,0,0,1] x[3,0,1,0] x[3,0,1,1] x[3,0,2,0] x[3,0,2,1]
 x[3,0,3,0] x[3,0,3,1] x[3,1,0,0] x[3,1,0,1] x[3,1,1,0] x[3,1,1,1]
 x[3,1,2,0] x[3,1,2,1] x[3,1,3,0] x[3,1,3,1] x[3,2,0,0] x[3,2,0,1]
 x[3,2,1,0] x[3,2,1,1] x[3,2,2,0] x[3,2,2,1] x[3,2,3,0] x[3,2,3,1]
 x[3,3,0,0] x[3,3,0,1] x[3,3,1,0] x[3,3,1,1] x[3,3,2,0] x[3,3,2,1]
 x[3,3,3,0] x[3,3,3,1] a[0,0] a[0,1] a[1,0] a[1,1] a[2,0] a[2,1] a[3,0]
 a[3,1]
End
