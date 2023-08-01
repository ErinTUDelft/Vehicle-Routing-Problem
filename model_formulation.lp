\ LP format - for model browsing. Use MPS format to capture full model detail.
Minimize
  T + 0 x[0,0,0,0] + 0 x[0,0,0,1] + 0 x[0,0,1,0] + 0 x[0,0,1,1]
   + 0 x[0,0,2,0] + 0 x[0,0,2,1] + 0 x[0,0,3,0] + 0 x[0,0,3,1]
   + 0 x[0,0,4,0] + 0 x[0,0,4,1] + 0 x[1,1,0,0] + 0 x[1,1,0,1]
   + 0 x[1,1,1,0] + 0 x[1,1,1,1] + 0 x[1,1,2,0] + 0 x[1,1,2,1]
   + 0 x[1,1,3,0] + 0 x[1,1,3,1] + 0 x[1,1,4,0] + 0 x[1,1,4,1]
   + 0 x[2,2,0,0] + 0 x[2,2,0,1] + 0 x[2,2,1,0] + 0 x[2,2,1,1]
   + 0 x[2,2,2,0] + 0 x[2,2,2,1] + 0 x[2,2,3,0] + 0 x[2,2,3,1]
   + 0 x[2,2,4,0] + 0 x[2,2,4,1] + 0 x[3,3,0,0] + 0 x[3,3,0,1]
   + 0 x[3,3,1,0] + 0 x[3,3,1,1] + 0 x[3,3,2,0] + 0 x[3,3,2,1]
   + 0 x[3,3,3,0] + 0 x[3,3,3,1] + 0 x[3,3,4,0] + 0 x[3,3,4,1]
   + 0 x[4,4,0,0] + 0 x[4,4,0,1] + 0 x[4,4,1,0] + 0 x[4,4,1,1]
   + 0 x[4,4,2,0] + 0 x[4,4,2,1] + 0 x[4,4,3,0] + 0 x[4,4,3,1]
   + 0 x[4,4,4,0] + 0 x[4,4,4,1] + 0 x[5,5,0,0] + 0 x[5,5,0,1]
   + 0 x[5,5,1,0] + 0 x[5,5,1,1] + 0 x[5,5,2,0] + 0 x[5,5,2,1]
   + 0 x[5,5,3,0] + 0 x[5,5,3,1] + 0 x[5,5,4,0] + 0 x[5,5,4,1]
   + 0 x[6,6,0,0] + 0 x[6,6,0,1] + 0 x[6,6,1,0] + 0 x[6,6,1,1]
   + 0 x[6,6,2,0] + 0 x[6,6,2,1] + 0 x[6,6,3,0] + 0 x[6,6,3,1]
   + 0 x[6,6,4,0] + 0 x[6,6,4,1] + arr[0,0,1] + arr[0,1,1] + arr[0,2,1]
   + arr[0,3,1] + arr[0,4,1] + 1.786305985935061 Tcomp[1]
   + 1.866289298581699 Tcomp[2] + 1.173165421495948 Tcomp[3]
   + 1.074948587013107 Tcomp[4] + 1.600742721377769 Tcomp[5]
   + 1.167972183718404 Tcomp[6]
Subject To
 node_1_satisfy_pesticide: p[1,0,0] + p[1,0,1] + p[1,1,0] + p[1,1,1]
   + p[1,2,0] + p[1,2,1] + p[1,3,0] + p[1,3,1] + p[1,4,0] + p[1,4,1]
   >= 10.37600549539573
 node_2_satisfy_pesticide: p[2,0,0] + p[2,0,1] + p[2,1,0] + p[2,1,1]
   + p[2,2,0] + p[2,2,1] + p[2,3,0] + p[2,3,1] + p[2,4,0] + p[2,4,1]
   >= 5.080874879007995
 node_3_satisfy_pesticide: p[3,0,0] + p[3,0,1] + p[3,1,0] + p[3,1,1]
   + p[3,2,0] + p[3,2,1] + p[3,3,0] + p[3,3,1] + p[3,4,0] + p[3,4,1]
   >= 8.479141543792664
 node_4_satisfy_pesticide: p[4,0,0] + p[4,0,1] + p[4,1,0] + p[4,1,1]
   + p[4,2,0] + p[4,2,1] + p[4,3,0] + p[4,3,1] + p[4,4,0] + p[4,4,1]
   >= 5.397440521383496
 node_5_satisfy_pesticide: p[5,0,0] + p[5,0,1] + p[5,1,0] + p[5,1,1]
   + p[5,2,0] + p[5,2,1] + p[5,3,0] + p[5,3,1] + p[5,4,0] + p[5,4,1]
   >= 13.60987670942812
 node_6_satisfy_pesticide: p[6,0,0] + p[6,0,1] + p[6,1,0] + p[6,1,1]
   + p[6,2,0] + p[6,2,1] + p[6,3,0] + p[6,3,1] + p[6,4,0] + p[6,4,1]
   >= 8.92531982191818
 drone_0_can_drop_at_0_in_trip_0: - 3600 x[0,1,0,0] - 3600 x[0,2,0,0]
   - 3600 x[0,3,0,0] - 3600 x[0,4,0,0] - 3600 x[0,5,0,0] - 3600 x[0,6,0,0]
   + p[0,0,0] <= 0
 drone_0_can_drop_at_0_in_trip_1: - 3600 x[0,1,0,1] - 3600 x[0,2,0,1]
   - 3600 x[0,3,0,1] - 3600 x[0,4,0,1] - 3600 x[0,5,0,1] - 3600 x[0,6,0,1]
   + p[0,0,1] <= 0
 drone_1_can_drop_at_0_in_trip_0: - 3600 x[0,1,1,0] - 3600 x[0,2,1,0]
   - 3600 x[0,3,1,0] - 3600 x[0,4,1,0] - 3600 x[0,5,1,0] - 3600 x[0,6,1,0]
   + p[0,1,0] <= 0
 drone_1_can_drop_at_0_in_trip_1: - 3600 x[0,1,1,1] - 3600 x[0,2,1,1]
   - 3600 x[0,3,1,1] - 3600 x[0,4,1,1] - 3600 x[0,5,1,1] - 3600 x[0,6,1,1]
   + p[0,1,1] <= 0
 drone_2_can_drop_at_0_in_trip_0: - 3600 x[0,1,2,0] - 3600 x[0,2,2,0]
   - 3600 x[0,3,2,0] - 3600 x[0,4,2,0] - 3600 x[0,5,2,0] - 3600 x[0,6,2,0]
   + p[0,2,0] <= 0
 drone_2_can_drop_at_0_in_trip_1: - 3600 x[0,1,2,1] - 3600 x[0,2,2,1]
   - 3600 x[0,3,2,1] - 3600 x[0,4,2,1] - 3600 x[0,5,2,1] - 3600 x[0,6,2,1]
   + p[0,2,1] <= 0
 drone_3_can_drop_at_0_in_trip_0: - 3600 x[0,1,3,0] - 3600 x[0,2,3,0]
   - 3600 x[0,3,3,0] - 3600 x[0,4,3,0] - 3600 x[0,5,3,0] - 3600 x[0,6,3,0]
   + p[0,3,0] <= 0
 drone_3_can_drop_at_0_in_trip_1: - 3600 x[0,1,3,1] - 3600 x[0,2,3,1]
   - 3600 x[0,3,3,1] - 3600 x[0,4,3,1] - 3600 x[0,5,3,1] - 3600 x[0,6,3,1]
   + p[0,3,1] <= 0
 drone_4_can_drop_at_0_in_trip_0: - 3600 x[0,1,4,0] - 3600 x[0,2,4,0]
   - 3600 x[0,3,4,0] - 3600 x[0,4,4,0] - 3600 x[0,5,4,0] - 3600 x[0,6,4,0]
   + p[0,4,0] <= 0
 drone_4_can_drop_at_0_in_trip_1: - 3600 x[0,1,4,1] - 3600 x[0,2,4,1]
   - 3600 x[0,3,4,1] - 3600 x[0,4,4,1] - 3600 x[0,5,4,1] - 3600 x[0,6,4,1]
   + p[0,4,1] <= 0
 drone_0_can_drop_at_1_in_trip_0: - 3600 x[1,0,0,0] - 3600 x[1,2,0,0]
   - 3600 x[1,3,0,0] - 3600 x[1,4,0,0] - 3600 x[1,5,0,0] - 3600 x[1,6,0,0]
   + p[1,0,0] <= 0
 drone_0_can_drop_at_1_in_trip_1: - 3600 x[1,0,0,1] - 3600 x[1,2,0,1]
   - 3600 x[1,3,0,1] - 3600 x[1,4,0,1] - 3600 x[1,5,0,1] - 3600 x[1,6,0,1]
   + p[1,0,1] <= 0
 drone_1_can_drop_at_1_in_trip_0: - 3600 x[1,0,1,0] - 3600 x[1,2,1,0]
   - 3600 x[1,3,1,0] - 3600 x[1,4,1,0] - 3600 x[1,5,1,0] - 3600 x[1,6,1,0]
   + p[1,1,0] <= 0
 drone_1_can_drop_at_1_in_trip_1: - 3600 x[1,0,1,1] - 3600 x[1,2,1,1]
   - 3600 x[1,3,1,1] - 3600 x[1,4,1,1] - 3600 x[1,5,1,1] - 3600 x[1,6,1,1]
   + p[1,1,1] <= 0
 drone_2_can_drop_at_1_in_trip_0: - 3600 x[1,0,2,0] - 3600 x[1,2,2,0]
   - 3600 x[1,3,2,0] - 3600 x[1,4,2,0] - 3600 x[1,5,2,0] - 3600 x[1,6,2,0]
   + p[1,2,0] <= 0
 drone_2_can_drop_at_1_in_trip_1: - 3600 x[1,0,2,1] - 3600 x[1,2,2,1]
   - 3600 x[1,3,2,1] - 3600 x[1,4,2,1] - 3600 x[1,5,2,1] - 3600 x[1,6,2,1]
   + p[1,2,1] <= 0
 drone_3_can_drop_at_1_in_trip_0: - 3600 x[1,0,3,0] - 3600 x[1,2,3,0]
   - 3600 x[1,3,3,0] - 3600 x[1,4,3,0] - 3600 x[1,5,3,0] - 3600 x[1,6,3,0]
   + p[1,3,0] <= 0
 drone_3_can_drop_at_1_in_trip_1: - 3600 x[1,0,3,1] - 3600 x[1,2,3,1]
   - 3600 x[1,3,3,1] - 3600 x[1,4,3,1] - 3600 x[1,5,3,1] - 3600 x[1,6,3,1]
   + p[1,3,1] <= 0
 drone_4_can_drop_at_1_in_trip_0: - 3600 x[1,0,4,0] - 3600 x[1,2,4,0]
   - 3600 x[1,3,4,0] - 3600 x[1,4,4,0] - 3600 x[1,5,4,0] - 3600 x[1,6,4,0]
   + p[1,4,0] <= 0
 drone_4_can_drop_at_1_in_trip_1: - 3600 x[1,0,4,1] - 3600 x[1,2,4,1]
   - 3600 x[1,3,4,1] - 3600 x[1,4,4,1] - 3600 x[1,5,4,1] - 3600 x[1,6,4,1]
   + p[1,4,1] <= 0
 drone_0_can_drop_at_2_in_trip_0: - 3600 x[2,0,0,0] - 3600 x[2,1,0,0]
   - 3600 x[2,3,0,0] - 3600 x[2,4,0,0] - 3600 x[2,5,0,0] - 3600 x[2,6,0,0]
   + p[2,0,0] <= 0
 drone_0_can_drop_at_2_in_trip_1: - 3600 x[2,0,0,1] - 3600 x[2,1,0,1]
   - 3600 x[2,3,0,1] - 3600 x[2,4,0,1] - 3600 x[2,5,0,1] - 3600 x[2,6,0,1]
   + p[2,0,1] <= 0
 drone_1_can_drop_at_2_in_trip_0: - 3600 x[2,0,1,0] - 3600 x[2,1,1,0]
   - 3600 x[2,3,1,0] - 3600 x[2,4,1,0] - 3600 x[2,5,1,0] - 3600 x[2,6,1,0]
   + p[2,1,0] <= 0
 drone_1_can_drop_at_2_in_trip_1: - 3600 x[2,0,1,1] - 3600 x[2,1,1,1]
   - 3600 x[2,3,1,1] - 3600 x[2,4,1,1] - 3600 x[2,5,1,1] - 3600 x[2,6,1,1]
   + p[2,1,1] <= 0
 drone_2_can_drop_at_2_in_trip_0: - 3600 x[2,0,2,0] - 3600 x[2,1,2,0]
   - 3600 x[2,3,2,0] - 3600 x[2,4,2,0] - 3600 x[2,5,2,0] - 3600 x[2,6,2,0]
   + p[2,2,0] <= 0
 drone_2_can_drop_at_2_in_trip_1: - 3600 x[2,0,2,1] - 3600 x[2,1,2,1]
   - 3600 x[2,3,2,1] - 3600 x[2,4,2,1] - 3600 x[2,5,2,1] - 3600 x[2,6,2,1]
   + p[2,2,1] <= 0
 drone_3_can_drop_at_2_in_trip_0: - 3600 x[2,0,3,0] - 3600 x[2,1,3,0]
   - 3600 x[2,3,3,0] - 3600 x[2,4,3,0] - 3600 x[2,5,3,0] - 3600 x[2,6,3,0]
   + p[2,3,0] <= 0
 drone_3_can_drop_at_2_in_trip_1: - 3600 x[2,0,3,1] - 3600 x[2,1,3,1]
   - 3600 x[2,3,3,1] - 3600 x[2,4,3,1] - 3600 x[2,5,3,1] - 3600 x[2,6,3,1]
   + p[2,3,1] <= 0
 drone_4_can_drop_at_2_in_trip_0: - 3600 x[2,0,4,0] - 3600 x[2,1,4,0]
   - 3600 x[2,3,4,0] - 3600 x[2,4,4,0] - 3600 x[2,5,4,0] - 3600 x[2,6,4,0]
   + p[2,4,0] <= 0
 drone_4_can_drop_at_2_in_trip_1: - 3600 x[2,0,4,1] - 3600 x[2,1,4,1]
   - 3600 x[2,3,4,1] - 3600 x[2,4,4,1] - 3600 x[2,5,4,1] - 3600 x[2,6,4,1]
   + p[2,4,1] <= 0
 drone_0_can_drop_at_3_in_trip_0: - 3600 x[3,0,0,0] - 3600 x[3,1,0,0]
   - 3600 x[3,2,0,0] - 3600 x[3,4,0,0] - 3600 x[3,5,0,0] - 3600 x[3,6,0,0]
   + p[3,0,0] <= 0
 drone_0_can_drop_at_3_in_trip_1: - 3600 x[3,0,0,1] - 3600 x[3,1,0,1]
   - 3600 x[3,2,0,1] - 3600 x[3,4,0,1] - 3600 x[3,5,0,1] - 3600 x[3,6,0,1]
   + p[3,0,1] <= 0
 drone_1_can_drop_at_3_in_trip_0: - 3600 x[3,0,1,0] - 3600 x[3,1,1,0]
   - 3600 x[3,2,1,0] - 3600 x[3,4,1,0] - 3600 x[3,5,1,0] - 3600 x[3,6,1,0]
   + p[3,1,0] <= 0
 drone_1_can_drop_at_3_in_trip_1: - 3600 x[3,0,1,1] - 3600 x[3,1,1,1]
   - 3600 x[3,2,1,1] - 3600 x[3,4,1,1] - 3600 x[3,5,1,1] - 3600 x[3,6,1,1]
   + p[3,1,1] <= 0
 drone_2_can_drop_at_3_in_trip_0: - 3600 x[3,0,2,0] - 3600 x[3,1,2,0]
   - 3600 x[3,2,2,0] - 3600 x[3,4,2,0] - 3600 x[3,5,2,0] - 3600 x[3,6,2,0]
   + p[3,2,0] <= 0
 drone_2_can_drop_at_3_in_trip_1: - 3600 x[3,0,2,1] - 3600 x[3,1,2,1]
   - 3600 x[3,2,2,1] - 3600 x[3,4,2,1] - 3600 x[3,5,2,1] - 3600 x[3,6,2,1]
   + p[3,2,1] <= 0
 drone_3_can_drop_at_3_in_trip_0: - 3600 x[3,0,3,0] - 3600 x[3,1,3,0]
   - 3600 x[3,2,3,0] - 3600 x[3,4,3,0] - 3600 x[3,5,3,0] - 3600 x[3,6,3,0]
   + p[3,3,0] <= 0
 drone_3_can_drop_at_3_in_trip_1: - 3600 x[3,0,3,1] - 3600 x[3,1,3,1]
   - 3600 x[3,2,3,1] - 3600 x[3,4,3,1] - 3600 x[3,5,3,1] - 3600 x[3,6,3,1]
   + p[3,3,1] <= 0
 drone_4_can_drop_at_3_in_trip_0: - 3600 x[3,0,4,0] - 3600 x[3,1,4,0]
   - 3600 x[3,2,4,0] - 3600 x[3,4,4,0] - 3600 x[3,5,4,0] - 3600 x[3,6,4,0]
   + p[3,4,0] <= 0
 drone_4_can_drop_at_3_in_trip_1: - 3600 x[3,0,4,1] - 3600 x[3,1,4,1]
   - 3600 x[3,2,4,1] - 3600 x[3,4,4,1] - 3600 x[3,5,4,1] - 3600 x[3,6,4,1]
   + p[3,4,1] <= 0
 drone_0_can_drop_at_4_in_trip_0: - 3600 x[4,0,0,0] - 3600 x[4,1,0,0]
   - 3600 x[4,2,0,0] - 3600 x[4,3,0,0] - 3600 x[4,5,0,0] - 3600 x[4,6,0,0]
   + p[4,0,0] <= 0
 drone_0_can_drop_at_4_in_trip_1: - 3600 x[4,0,0,1] - 3600 x[4,1,0,1]
   - 3600 x[4,2,0,1] - 3600 x[4,3,0,1] - 3600 x[4,5,0,1] - 3600 x[4,6,0,1]
   + p[4,0,1] <= 0
 drone_1_can_drop_at_4_in_trip_0: - 3600 x[4,0,1,0] - 3600 x[4,1,1,0]
   - 3600 x[4,2,1,0] - 3600 x[4,3,1,0] - 3600 x[4,5,1,0] - 3600 x[4,6,1,0]
   + p[4,1,0] <= 0
 drone_1_can_drop_at_4_in_trip_1: - 3600 x[4,0,1,1] - 3600 x[4,1,1,1]
   - 3600 x[4,2,1,1] - 3600 x[4,3,1,1] - 3600 x[4,5,1,1] - 3600 x[4,6,1,1]
   + p[4,1,1] <= 0
 drone_2_can_drop_at_4_in_trip_0: - 3600 x[4,0,2,0] - 3600 x[4,1,2,0]
   - 3600 x[4,2,2,0] - 3600 x[4,3,2,0] - 3600 x[4,5,2,0] - 3600 x[4,6,2,0]
   + p[4,2,0] <= 0
 drone_2_can_drop_at_4_in_trip_1: - 3600 x[4,0,2,1] - 3600 x[4,1,2,1]
   - 3600 x[4,2,2,1] - 3600 x[4,3,2,1] - 3600 x[4,5,2,1] - 3600 x[4,6,2,1]
   + p[4,2,1] <= 0
 drone_3_can_drop_at_4_in_trip_0: - 3600 x[4,0,3,0] - 3600 x[4,1,3,0]
   - 3600 x[4,2,3,0] - 3600 x[4,3,3,0] - 3600 x[4,5,3,0] - 3600 x[4,6,3,0]
   + p[4,3,0] <= 0
 drone_3_can_drop_at_4_in_trip_1: - 3600 x[4,0,3,1] - 3600 x[4,1,3,1]
   - 3600 x[4,2,3,1] - 3600 x[4,3,3,1] - 3600 x[4,5,3,1] - 3600 x[4,6,3,1]
   + p[4,3,1] <= 0
 drone_4_can_drop_at_4_in_trip_0: - 3600 x[4,0,4,0] - 3600 x[4,1,4,0]
   - 3600 x[4,2,4,0] - 3600 x[4,3,4,0] - 3600 x[4,5,4,0] - 3600 x[4,6,4,0]
   + p[4,4,0] <= 0
 drone_4_can_drop_at_4_in_trip_1: - 3600 x[4,0,4,1] - 3600 x[4,1,4,1]
   - 3600 x[4,2,4,1] - 3600 x[4,3,4,1] - 3600 x[4,5,4,1] - 3600 x[4,6,4,1]
   + p[4,4,1] <= 0
 drone_0_can_drop_at_5_in_trip_0: - 3600 x[5,0,0,0] - 3600 x[5,1,0,0]
   - 3600 x[5,2,0,0] - 3600 x[5,3,0,0] - 3600 x[5,4,0,0] - 3600 x[5,6,0,0]
   + p[5,0,0] <= 0
 drone_0_can_drop_at_5_in_trip_1: - 3600 x[5,0,0,1] - 3600 x[5,1,0,1]
   - 3600 x[5,2,0,1] - 3600 x[5,3,0,1] - 3600 x[5,4,0,1] - 3600 x[5,6,0,1]
   + p[5,0,1] <= 0
 drone_1_can_drop_at_5_in_trip_0: - 3600 x[5,0,1,0] - 3600 x[5,1,1,0]
   - 3600 x[5,2,1,0] - 3600 x[5,3,1,0] - 3600 x[5,4,1,0] - 3600 x[5,6,1,0]
   + p[5,1,0] <= 0
 drone_1_can_drop_at_5_in_trip_1: - 3600 x[5,0,1,1] - 3600 x[5,1,1,1]
   - 3600 x[5,2,1,1] - 3600 x[5,3,1,1] - 3600 x[5,4,1,1] - 3600 x[5,6,1,1]
   + p[5,1,1] <= 0
 drone_2_can_drop_at_5_in_trip_0: - 3600 x[5,0,2,0] - 3600 x[5,1,2,0]
   - 3600 x[5,2,2,0] - 3600 x[5,3,2,0] - 3600 x[5,4,2,0] - 3600 x[5,6,2,0]
   + p[5,2,0] <= 0
 drone_2_can_drop_at_5_in_trip_1: - 3600 x[5,0,2,1] - 3600 x[5,1,2,1]
   - 3600 x[5,2,2,1] - 3600 x[5,3,2,1] - 3600 x[5,4,2,1] - 3600 x[5,6,2,1]
   + p[5,2,1] <= 0
 drone_3_can_drop_at_5_in_trip_0: - 3600 x[5,0,3,0] - 3600 x[5,1,3,0]
   - 3600 x[5,2,3,0] - 3600 x[5,3,3,0] - 3600 x[5,4,3,0] - 3600 x[5,6,3,0]
   + p[5,3,0] <= 0
 drone_3_can_drop_at_5_in_trip_1: - 3600 x[5,0,3,1] - 3600 x[5,1,3,1]
   - 3600 x[5,2,3,1] - 3600 x[5,3,3,1] - 3600 x[5,4,3,1] - 3600 x[5,6,3,1]
   + p[5,3,1] <= 0
 drone_4_can_drop_at_5_in_trip_0: - 3600 x[5,0,4,0] - 3600 x[5,1,4,0]
   - 3600 x[5,2,4,0] - 3600 x[5,3,4,0] - 3600 x[5,4,4,0] - 3600 x[5,6,4,0]
   + p[5,4,0] <= 0
 drone_4_can_drop_at_5_in_trip_1: - 3600 x[5,0,4,1] - 3600 x[5,1,4,1]
   - 3600 x[5,2,4,1] - 3600 x[5,3,4,1] - 3600 x[5,4,4,1] - 3600 x[5,6,4,1]
   + p[5,4,1] <= 0
 drone_0_can_drop_at_6_in_trip_0: - 3600 x[6,0,0,0] - 3600 x[6,1,0,0]
   - 3600 x[6,2,0,0] - 3600 x[6,3,0,0] - 3600 x[6,4,0,0] - 3600 x[6,5,0,0]
   + p[6,0,0] <= 0
 drone_0_can_drop_at_6_in_trip_1: - 3600 x[6,0,0,1] - 3600 x[6,1,0,1]
   - 3600 x[6,2,0,1] - 3600 x[6,3,0,1] - 3600 x[6,4,0,1] - 3600 x[6,5,0,1]
   + p[6,0,1] <= 0
 drone_1_can_drop_at_6_in_trip_0: - 3600 x[6,0,1,0] - 3600 x[6,1,1,0]
   - 3600 x[6,2,1,0] - 3600 x[6,3,1,0] - 3600 x[6,4,1,0] - 3600 x[6,5,1,0]
   + p[6,1,0] <= 0
 drone_1_can_drop_at_6_in_trip_1: - 3600 x[6,0,1,1] - 3600 x[6,1,1,1]
   - 3600 x[6,2,1,1] - 3600 x[6,3,1,1] - 3600 x[6,4,1,1] - 3600 x[6,5,1,1]
   + p[6,1,1] <= 0
 drone_2_can_drop_at_6_in_trip_0: - 3600 x[6,0,2,0] - 3600 x[6,1,2,0]
   - 3600 x[6,2,2,0] - 3600 x[6,3,2,0] - 3600 x[6,4,2,0] - 3600 x[6,5,2,0]
   + p[6,2,0] <= 0
 drone_2_can_drop_at_6_in_trip_1: - 3600 x[6,0,2,1] - 3600 x[6,1,2,1]
   - 3600 x[6,2,2,1] - 3600 x[6,3,2,1] - 3600 x[6,4,2,1] - 3600 x[6,5,2,1]
   + p[6,2,1] <= 0
 drone_3_can_drop_at_6_in_trip_0: - 3600 x[6,0,3,0] - 3600 x[6,1,3,0]
   - 3600 x[6,2,3,0] - 3600 x[6,3,3,0] - 3600 x[6,4,3,0] - 3600 x[6,5,3,0]
   + p[6,3,0] <= 0
 drone_3_can_drop_at_6_in_trip_1: - 3600 x[6,0,3,1] - 3600 x[6,1,3,1]
   - 3600 x[6,2,3,1] - 3600 x[6,3,3,1] - 3600 x[6,4,3,1] - 3600 x[6,5,3,1]
   + p[6,3,1] <= 0
 drone_4_can_drop_at_6_in_trip_0: - 3600 x[6,0,4,0] - 3600 x[6,1,4,0]
   - 3600 x[6,2,4,0] - 3600 x[6,3,4,0] - 3600 x[6,4,4,0] - 3600 x[6,5,4,0]
   + p[6,4,0] <= 0
 drone_4_can_drop_at_6_in_trip_1: - 3600 x[6,0,4,1] - 3600 x[6,1,4,1]
   - 3600 x[6,2,4,1] - 3600 x[6,3,4,1] - 3600 x[6,4,4,1] - 3600 x[6,5,4,1]
   + p[6,4,1] <= 0
 drone_0_can_carry_amount_of_pesticide_in_trip_0: p[0,0,0] + p[1,0,0]
   + p[2,0,0] + p[3,0,0] + p[4,0,0] + p[5,0,0] + p[6,0,0] <= 7
 drone_0_can_carry_amount_of_pesticide_in_trip_1: p[0,0,1] + p[1,0,1]
   + p[2,0,1] + p[3,0,1] + p[4,0,1] + p[5,0,1] + p[6,0,1] <= 7
 drone_1_can_carry_amount_of_pesticide_in_trip_0: p[0,1,0] + p[1,1,0]
   + p[2,1,0] + p[3,1,0] + p[4,1,0] + p[5,1,0] + p[6,1,0] <= 7
 drone_1_can_carry_amount_of_pesticide_in_trip_1: p[0,1,1] + p[1,1,1]
   + p[2,1,1] + p[3,1,1] + p[4,1,1] + p[5,1,1] + p[6,1,1] <= 7
 drone_2_can_carry_amount_of_pesticide_in_trip_0: p[0,2,0] + p[1,2,0]
   + p[2,2,0] + p[3,2,0] + p[4,2,0] + p[5,2,0] + p[6,2,0] <= 7
 drone_2_can_carry_amount_of_pesticide_in_trip_1: p[0,2,1] + p[1,2,1]
   + p[2,2,1] + p[3,2,1] + p[4,2,1] + p[5,2,1] + p[6,2,1] <= 7
 drone_3_can_carry_amount_of_pesticide_in_trip_0: p[0,3,0] + p[1,3,0]
   + p[2,3,0] + p[3,3,0] + p[4,3,0] + p[5,3,0] + p[6,3,0] <= 7
 drone_3_can_carry_amount_of_pesticide_in_trip_1: p[0,3,1] + p[1,3,1]
   + p[2,3,1] + p[3,3,1] + p[4,3,1] + p[5,3,1] + p[6,3,1] <= 7
 drone_4_can_carry_amount_of_pesticide_in_trip_0: p[0,4,0] + p[1,4,0]
   + p[2,4,0] + p[3,4,0] + p[4,4,0] + p[5,4,0] + p[6,4,0] <= 7
 drone_4_can_carry_amount_of_pesticide_in_trip_1: p[0,4,1] + p[1,4,1]
   + p[2,4,1] + p[3,4,1] + p[4,4,1] + p[5,4,1] + p[6,4,1] <= 7
 drone_0_starts_at_origin: dep[0,0,0] = 0
 drone_1_starts_at_origin: dep[0,1,0] = 0
 drone_2_starts_at_origin: dep[0,2,0] = 0
 drone_3_starts_at_origin: dep[0,3,0] = 0
 drone_4_starts_at_origin: dep[0,4,0] = 0
 travel_time_from0to1_drone0_trip0: 3600 x[0,1,0,0] + dep[0,0,0]
   - arr[1,0,0] <= 3558.084543206373
 travel_time_from0to1_drone0_trip1: 3600 x[0,1,0,1] + dep[0,0,1]
   - arr[1,0,1] <= 3558.084543206373
 travel_time_from0to1_drone1_trip0: 3600 x[0,1,1,0] + dep[0,1,0]
   - arr[1,1,0] <= 3558.084543206373
 travel_time_from0to1_drone1_trip1: 3600 x[0,1,1,1] + dep[0,1,1]
   - arr[1,1,1] <= 3558.084543206373
 travel_time_from0to1_drone2_trip0: 3600 x[0,1,2,0] + dep[0,2,0]
   - arr[1,2,0] <= 3558.084543206373
 travel_time_from0to1_drone2_trip1: 3600 x[0,1,2,1] + dep[0,2,1]
   - arr[1,2,1] <= 3558.084543206373
 travel_time_from0to1_drone3_trip0: 3600 x[0,1,3,0] + dep[0,3,0]
   - arr[1,3,0] <= 3558.084543206373
 travel_time_from0to1_drone3_trip1: 3600 x[0,1,3,1] + dep[0,3,1]
   - arr[1,3,1] <= 3558.084543206373
 travel_time_from0to1_drone4_trip0: 3600 x[0,1,4,0] + dep[0,4,0]
   - arr[1,4,0] <= 3558.084543206373
 travel_time_from0to1_drone4_trip1: 3600 x[0,1,4,1] + dep[0,4,1]
   - arr[1,4,1] <= 3558.084543206373
 travel_time_from0to2_drone0_trip0: 3600 x[0,2,0,0] + dep[0,0,0]
   - arr[2,0,0] <= 3520.473161692704
 travel_time_from0to2_drone0_trip1: 3600 x[0,2,0,1] + dep[0,0,1]
   - arr[2,0,1] <= 3520.473161692704
 travel_time_from0to2_drone1_trip0: 3600 x[0,2,1,0] + dep[0,1,0]
   - arr[2,1,0] <= 3520.473161692704
 travel_time_from0to2_drone1_trip1: 3600 x[0,2,1,1] + dep[0,1,1]
   - arr[2,1,1] <= 3520.473161692704
 travel_time_from0to2_drone2_trip0: 3600 x[0,2,2,0] + dep[0,2,0]
   - arr[2,2,0] <= 3520.473161692704
 travel_time_from0to2_drone2_trip1: 3600 x[0,2,2,1] + dep[0,2,1]
   - arr[2,2,1] <= 3520.473161692704
 travel_time_from0to2_drone3_trip0: 3600 x[0,2,3,0] + dep[0,3,0]
   - arr[2,3,0] <= 3520.473161692704
 travel_time_from0to2_drone3_trip1: 3600 x[0,2,3,1] + dep[0,3,1]
   - arr[2,3,1] <= 3520.473161692704
 travel_time_from0to2_drone4_trip0: 3600 x[0,2,4,0] + dep[0,4,0]
   - arr[2,4,0] <= 3520.473161692704
 travel_time_from0to2_drone4_trip1: 3600 x[0,2,4,1] + dep[0,4,1]
   - arr[2,4,1] <= 3520.473161692704
 travel_time_from0to3_drone0_trip0: 3600 x[0,3,0,0] + dep[0,0,0]
   - arr[3,0,0] <= 3541.263127204569
 travel_time_from0to3_drone0_trip1: 3600 x[0,3,0,1] + dep[0,0,1]
   - arr[3,0,1] <= 3541.263127204569
 travel_time_from0to3_drone1_trip0: 3600 x[0,3,1,0] + dep[0,1,0]
   - arr[3,1,0] <= 3541.263127204569
 travel_time_from0to3_drone1_trip1: 3600 x[0,3,1,1] + dep[0,1,1]
   - arr[3,1,1] <= 3541.263127204569
 travel_time_from0to3_drone2_trip0: 3600 x[0,3,2,0] + dep[0,2,0]
   - arr[3,2,0] <= 3541.263127204569
 travel_time_from0to3_drone2_trip1: 3600 x[0,3,2,1] + dep[0,2,1]
   - arr[3,2,1] <= 3541.263127204569
 travel_time_from0to3_drone3_trip0: 3600 x[0,3,3,0] + dep[0,3,0]
   - arr[3,3,0] <= 3541.263127204569
 travel_time_from0to3_drone3_trip1: 3600 x[0,3,3,1] + dep[0,3,1]
   - arr[3,3,1] <= 3541.263127204569
 travel_time_from0to3_drone4_trip0: 3600 x[0,3,4,0] + dep[0,4,0]
   - arr[3,4,0] <= 3541.263127204569
 travel_time_from0to3_drone4_trip1: 3600 x[0,3,4,1] + dep[0,4,1]
   - arr[3,4,1] <= 3541.263127204569
 travel_time_from0to4_drone0_trip0: 3600 x[0,4,0,0] + dep[0,0,0]
   - arr[4,0,0] <= 3539.79410277222
 travel_time_from0to4_drone0_trip1: 3600 x[0,4,0,1] + dep[0,0,1]
   - arr[4,0,1] <= 3539.79410277222
 travel_time_from0to4_drone1_trip0: 3600 x[0,4,1,0] + dep[0,1,0]
   - arr[4,1,0] <= 3539.79410277222
 travel_time_from0to4_drone1_trip1: 3600 x[0,4,1,1] + dep[0,1,1]
   - arr[4,1,1] <= 3539.79410277222
 travel_time_from0to4_drone2_trip0: 3600 x[0,4,2,0] + dep[0,2,0]
   - arr[4,2,0] <= 3539.79410277222
 travel_time_from0to4_drone2_trip1: 3600 x[0,4,2,1] + dep[0,2,1]
   - arr[4,2,1] <= 3539.79410277222
 travel_time_from0to4_drone3_trip0: 3600 x[0,4,3,0] + dep[0,3,0]
   - arr[4,3,0] <= 3539.79410277222
 travel_time_from0to4_drone3_trip1: 3600 x[0,4,3,1] + dep[0,3,1]
   - arr[4,3,1] <= 3539.79410277222
 travel_time_from0to4_drone4_trip0: 3600 x[0,4,4,0] + dep[0,4,0]
   - arr[4,4,0] <= 3539.79410277222
 travel_time_from0to4_drone4_trip1: 3600 x[0,4,4,1] + dep[0,4,1]
   - arr[4,4,1] <= 3539.79410277222
 travel_time_from0to5_drone0_trip0: 3600 x[0,5,0,0] + dep[0,0,0]
   - arr[5,0,0] <= 3523.194100412758
 travel_time_from0to5_drone0_trip1: 3600 x[0,5,0,1] + dep[0,0,1]
   - arr[5,0,1] <= 3523.194100412758
 travel_time_from0to5_drone1_trip0: 3600 x[0,5,1,0] + dep[0,1,0]
   - arr[5,1,0] <= 3523.194100412758
 travel_time_from0to5_drone1_trip1: 3600 x[0,5,1,1] + dep[0,1,1]
   - arr[5,1,1] <= 3523.194100412758
 travel_time_from0to5_drone2_trip0: 3600 x[0,5,2,0] + dep[0,2,0]
   - arr[5,2,0] <= 3523.194100412758
 travel_time_from0to5_drone2_trip1: 3600 x[0,5,2,1] + dep[0,2,1]
   - arr[5,2,1] <= 3523.194100412758
 travel_time_from0to5_drone3_trip0: 3600 x[0,5,3,0] + dep[0,3,0]
   - arr[5,3,0] <= 3523.194100412758
 travel_time_from0to5_drone3_trip1: 3600 x[0,5,3,1] + dep[0,3,1]
   - arr[5,3,1] <= 3523.194100412758
 travel_time_from0to5_drone4_trip0: 3600 x[0,5,4,0] + dep[0,4,0]
   - arr[5,4,0] <= 3523.194100412758
 travel_time_from0to5_drone4_trip1: 3600 x[0,5,4,1] + dep[0,4,1]
   - arr[5,4,1] <= 3523.194100412758
 travel_time_from0to6_drone0_trip0: 3600 x[0,6,0,0] + dep[0,0,0]
   - arr[6,0,0] <= 3486.898082960795
 travel_time_from0to6_drone0_trip1: 3600 x[0,6,0,1] + dep[0,0,1]
   - arr[6,0,1] <= 3486.898082960795
 travel_time_from0to6_drone1_trip0: 3600 x[0,6,1,0] + dep[0,1,0]
   - arr[6,1,0] <= 3486.898082960795
 travel_time_from0to6_drone1_trip1: 3600 x[0,6,1,1] + dep[0,1,1]
   - arr[6,1,1] <= 3486.898082960795
 travel_time_from0to6_drone2_trip0: 3600 x[0,6,2,0] + dep[0,2,0]
   - arr[6,2,0] <= 3486.898082960795
 travel_time_from0to6_drone2_trip1: 3600 x[0,6,2,1] + dep[0,2,1]
   - arr[6,2,1] <= 3486.898082960795
 travel_time_from0to6_drone3_trip0: 3600 x[0,6,3,0] + dep[0,3,0]
   - arr[6,3,0] <= 3486.898082960795
 travel_time_from0to6_drone3_trip1: 3600 x[0,6,3,1] + dep[0,3,1]
   - arr[6,3,1] <= 3486.898082960795
 travel_time_from0to6_drone4_trip0: 3600 x[0,6,4,0] + dep[0,4,0]
   - arr[6,4,0] <= 3486.898082960795
 travel_time_from0to6_drone4_trip1: 3600 x[0,6,4,1] + dep[0,4,1]
   - arr[6,4,1] <= 3486.898082960795
 travel_time_from1to0_drone0_trip0: 3600 x[1,0,0,0] - arr[0,0,0]
   + dep[1,0,0] <= 3558.084543206373
 travel_time_from1to0_drone0_trip1: 3600 x[1,0,0,1] - arr[0,0,1]
   + dep[1,0,1] <= 3558.084543206373
 travel_time_from1to0_drone1_trip0: 3600 x[1,0,1,0] - arr[0,1,0]
   + dep[1,1,0] <= 3558.084543206373
 travel_time_from1to0_drone1_trip1: 3600 x[1,0,1,1] - arr[0,1,1]
   + dep[1,1,1] <= 3558.084543206373
 travel_time_from1to0_drone2_trip0: 3600 x[1,0,2,0] - arr[0,2,0]
   + dep[1,2,0] <= 3558.084543206373
 travel_time_from1to0_drone2_trip1: 3600 x[1,0,2,1] - arr[0,2,1]
   + dep[1,2,1] <= 3558.084543206373
 travel_time_from1to0_drone3_trip0: 3600 x[1,0,3,0] - arr[0,3,0]
   + dep[1,3,0] <= 3558.084543206373
 travel_time_from1to0_drone3_trip1: 3600 x[1,0,3,1] - arr[0,3,1]
   + dep[1,3,1] <= 3558.084543206373
 travel_time_from1to0_drone4_trip0: 3600 x[1,0,4,0] - arr[0,4,0]
   + dep[1,4,0] <= 3558.084543206373
 travel_time_from1to0_drone4_trip1: 3600 x[1,0,4,1] - arr[0,4,1]
   + dep[1,4,1] <= 3558.084543206373
 travel_time_from1to2_drone0_trip0: 3600 x[1,2,0,0] + dep[1,0,0]
   - arr[2,0,0] <= 3522.888235355233
 travel_time_from1to2_drone0_trip1: 3600 x[1,2,0,1] + dep[1,0,1]
   - arr[2,0,1] <= 3522.888235355233
 travel_time_from1to2_drone1_trip0: 3600 x[1,2,1,0] + dep[1,1,0]
   - arr[2,1,0] <= 3522.888235355233
 travel_time_from1to2_drone1_trip1: 3600 x[1,2,1,1] + dep[1,1,1]
   - arr[2,1,1] <= 3522.888235355233
 travel_time_from1to2_drone2_trip0: 3600 x[1,2,2,0] + dep[1,2,0]
   - arr[2,2,0] <= 3522.888235355233
 travel_time_from1to2_drone2_trip1: 3600 x[1,2,2,1] + dep[1,2,1]
   - arr[2,2,1] <= 3522.888235355233
 travel_time_from1to2_drone3_trip0: 3600 x[1,2,3,0] + dep[1,3,0]
   - arr[2,3,0] <= 3522.888235355233
 travel_time_from1to2_drone3_trip1: 3600 x[1,2,3,1] + dep[1,3,1]
   - arr[2,3,1] <= 3522.888235355233
 travel_time_from1to2_drone4_trip0: 3600 x[1,2,4,0] + dep[1,4,0]
   - arr[2,4,0] <= 3522.888235355233
 travel_time_from1to2_drone4_trip1: 3600 x[1,2,4,1] + dep[1,4,1]
   - arr[2,4,1] <= 3522.888235355233
 travel_time_from1to3_drone0_trip0: 3600 x[1,3,0,0] + dep[1,0,0]
   - arr[3,0,0] <= 3507.927887506031
 travel_time_from1to3_drone0_trip1: 3600 x[1,3,0,1] + dep[1,0,1]
   - arr[3,0,1] <= 3507.927887506031
 travel_time_from1to3_drone1_trip0: 3600 x[1,3,1,0] + dep[1,1,0]
   - arr[3,1,0] <= 3507.927887506031
 travel_time_from1to3_drone1_trip1: 3600 x[1,3,1,1] + dep[1,1,1]
   - arr[3,1,1] <= 3507.927887506031
 travel_time_from1to3_drone2_trip0: 3600 x[1,3,2,0] + dep[1,2,0]
   - arr[3,2,0] <= 3507.927887506031
 travel_time_from1to3_drone2_trip1: 3600 x[1,3,2,1] + dep[1,2,1]
   - arr[3,2,1] <= 3507.927887506031
 travel_time_from1to3_drone3_trip0: 3600 x[1,3,3,0] + dep[1,3,0]
   - arr[3,3,0] <= 3507.927887506031
 travel_time_from1to3_drone3_trip1: 3600 x[1,3,3,1] + dep[1,3,1]
   - arr[3,3,1] <= 3507.927887506031
 travel_time_from1to3_drone4_trip0: 3600 x[1,3,4,0] + dep[1,4,0]
   - arr[3,4,0] <= 3507.927887506031
 travel_time_from1to3_drone4_trip1: 3600 x[1,3,4,1] + dep[1,4,1]
   - arr[3,4,1] <= 3507.927887506031
 travel_time_from1to4_drone0_trip0: 3600 x[1,4,0,0] + dep[1,0,0]
   - arr[4,0,0] <= 3573.277657470349
 travel_time_from1to4_drone0_trip1: 3600 x[1,4,0,1] + dep[1,0,1]
   - arr[4,0,1] <= 3573.277657470349
 travel_time_from1to4_drone1_trip0: 3600 x[1,4,1,0] + dep[1,1,0]
   - arr[4,1,0] <= 3573.277657470349
 travel_time_from1to4_drone1_trip1: 3600 x[1,4,1,1] + dep[1,1,1]
   - arr[4,1,1] <= 3573.277657470349
 travel_time_from1to4_drone2_trip0: 3600 x[1,4,2,0] + dep[1,2,0]
   - arr[4,2,0] <= 3573.277657470349
 travel_time_from1to4_drone2_trip1: 3600 x[1,4,2,1] + dep[1,2,1]
   - arr[4,2,1] <= 3573.277657470349
 travel_time_from1to4_drone3_trip0: 3600 x[1,4,3,0] + dep[1,3,0]
   - arr[4,3,0] <= 3573.277657470349
 travel_time_from1to4_drone3_trip1: 3600 x[1,4,3,1] + dep[1,3,1]
   - arr[4,3,1] <= 3573.277657470349
 travel_time_from1to4_drone4_trip0: 3600 x[1,4,4,0] + dep[1,4,0]
   - arr[4,4,0] <= 3573.277657470349
 travel_time_from1to4_drone4_trip1: 3600 x[1,4,4,1] + dep[1,4,1]
   - arr[4,4,1] <= 3573.277657470349
 travel_time_from1to5_drone0_trip0: 3600 x[1,5,0,0] + dep[1,0,0]
   - arr[5,0,0] <= 3484.317389583469
 travel_time_from1to5_drone0_trip1: 3600 x[1,5,0,1] + dep[1,0,1]
   - arr[5,0,1] <= 3484.317389583469
 travel_time_from1to5_drone1_trip0: 3600 x[1,5,1,0] + dep[1,1,0]
   - arr[5,1,0] <= 3484.317389583469
 travel_time_from1to5_drone1_trip1: 3600 x[1,5,1,1] + dep[1,1,1]
   - arr[5,1,1] <= 3484.317389583469
 travel_time_from1to5_drone2_trip0: 3600 x[1,5,2,0] + dep[1,2,0]
   - arr[5,2,0] <= 3484.317389583469
 travel_time_from1to5_drone2_trip1: 3600 x[1,5,2,1] + dep[1,2,1]
   - arr[5,2,1] <= 3484.317389583469
 travel_time_from1to5_drone3_trip0: 3600 x[1,5,3,0] + dep[1,3,0]
   - arr[5,3,0] <= 3484.317389583469
 travel_time_from1to5_drone3_trip1: 3600 x[1,5,3,1] + dep[1,3,1]
   - arr[5,3,1] <= 3484.317389583469
 travel_time_from1to5_drone4_trip0: 3600 x[1,5,4,0] + dep[1,4,0]
   - arr[5,4,0] <= 3484.317389583469
 travel_time_from1to5_drone4_trip1: 3600 x[1,5,4,1] + dep[1,4,1]
   - arr[5,4,1] <= 3484.317389583469
 travel_time_from1to6_drone0_trip0: 3600 x[1,6,0,0] + dep[1,0,0]
   - arr[6,0,0] <= 3458.815710587392
 travel_time_from1to6_drone0_trip1: 3600 x[1,6,0,1] + dep[1,0,1]
   - arr[6,0,1] <= 3458.815710587392
 travel_time_from1to6_drone1_trip0: 3600 x[1,6,1,0] + dep[1,1,0]
   - arr[6,1,0] <= 3458.815710587392
 travel_time_from1to6_drone1_trip1: 3600 x[1,6,1,1] + dep[1,1,1]
   - arr[6,1,1] <= 3458.815710587392
 travel_time_from1to6_drone2_trip0: 3600 x[1,6,2,0] + dep[1,2,0]
   - arr[6,2,0] <= 3458.815710587392
 travel_time_from1to6_drone2_trip1: 3600 x[1,6,2,1] + dep[1,2,1]
   - arr[6,2,1] <= 3458.815710587392
 travel_time_from1to6_drone3_trip0: 3600 x[1,6,3,0] + dep[1,3,0]
   - arr[6,3,0] <= 3458.815710587392
 travel_time_from1to6_drone3_trip1: 3600 x[1,6,3,1] + dep[1,3,1]
   - arr[6,3,1] <= 3458.815710587392
 travel_time_from1to6_drone4_trip0: 3600 x[1,6,4,0] + dep[1,4,0]
   - arr[6,4,0] <= 3458.815710587392
 travel_time_from1to6_drone4_trip1: 3600 x[1,6,4,1] + dep[1,4,1]
   - arr[6,4,1] <= 3458.815710587392
 travel_time_from2to0_drone0_trip0: 3600 x[2,0,0,0] - arr[0,0,0]
   + dep[2,0,0] <= 3520.473161692704
 travel_time_from2to0_drone0_trip1: 3600 x[2,0,0,1] - arr[0,0,1]
   + dep[2,0,1] <= 3520.473161692704
 travel_time_from2to0_drone1_trip0: 3600 x[2,0,1,0] - arr[0,1,0]
   + dep[2,1,0] <= 3520.473161692704
 travel_time_from2to0_drone1_trip1: 3600 x[2,0,1,1] - arr[0,1,1]
   + dep[2,1,1] <= 3520.473161692704
 travel_time_from2to0_drone2_trip0: 3600 x[2,0,2,0] - arr[0,2,0]
   + dep[2,2,0] <= 3520.473161692704
 travel_time_from2to0_drone2_trip1: 3600 x[2,0,2,1] - arr[0,2,1]
   + dep[2,2,1] <= 3520.473161692704
 travel_time_from2to0_drone3_trip0: 3600 x[2,0,3,0] - arr[0,3,0]
   + dep[2,3,0] <= 3520.473161692704
 travel_time_from2to0_drone3_trip1: 3600 x[2,0,3,1] - arr[0,3,1]
   + dep[2,3,1] <= 3520.473161692704
 travel_time_from2to0_drone4_trip0: 3600 x[2,0,4,0] - arr[0,4,0]
   + dep[2,4,0] <= 3520.473161692704
 travel_time_from2to0_drone4_trip1: 3600 x[2,0,4,1] - arr[0,4,1]
   + dep[2,4,1] <= 3520.473161692704
 travel_time_from2to1_drone0_trip0: 3600 x[2,1,0,0] - arr[1,0,0]
   + dep[2,0,0] <= 3522.888235355233
 travel_time_from2to1_drone0_trip1: 3600 x[2,1,0,1] - arr[1,0,1]
   + dep[2,0,1] <= 3522.888235355233
 travel_time_from2to1_drone1_trip0: 3600 x[2,1,1,0] - arr[1,1,0]
   + dep[2,1,0] <= 3522.888235355233
 travel_time_from2to1_drone1_trip1: 3600 x[2,1,1,1] - arr[1,1,1]
   + dep[2,1,1] <= 3522.888235355233
 travel_time_from2to1_drone2_trip0: 3600 x[2,1,2,0] - arr[1,2,0]
   + dep[2,2,0] <= 3522.888235355233
 travel_time_from2to1_drone2_trip1: 3600 x[2,1,2,1] - arr[1,2,1]
   + dep[2,2,1] <= 3522.888235355233
 travel_time_from2to1_drone3_trip0: 3600 x[2,1,3,0] - arr[1,3,0]
   + dep[2,3,0] <= 3522.888235355233
 travel_time_from2to1_drone3_trip1: 3600 x[2,1,3,1] - arr[1,3,1]
   + dep[2,3,1] <= 3522.888235355233
 travel_time_from2to1_drone4_trip0: 3600 x[2,1,4,0] - arr[1,4,0]
   + dep[2,4,0] <= 3522.888235355233
 travel_time_from2to1_drone4_trip1: 3600 x[2,1,4,1] - arr[1,4,1]
   + dep[2,4,1] <= 3522.888235355233
 travel_time_from2to3_drone0_trip0: 3600 x[2,3,0,0] + dep[2,0,0]
   - arr[3,0,0] <= 3528.26760379047
 travel_time_from2to3_drone0_trip1: 3600 x[2,3,0,1] + dep[2,0,1]
   - arr[3,0,1] <= 3528.26760379047
 travel_time_from2to3_drone1_trip0: 3600 x[2,3,1,0] + dep[2,1,0]
   - arr[3,1,0] <= 3528.26760379047
 travel_time_from2to3_drone1_trip1: 3600 x[2,3,1,1] + dep[2,1,1]
   - arr[3,1,1] <= 3528.26760379047
 travel_time_from2to3_drone2_trip0: 3600 x[2,3,2,0] + dep[2,2,0]
   - arr[3,2,0] <= 3528.26760379047
 travel_time_from2to3_drone2_trip1: 3600 x[2,3,2,1] + dep[2,2,1]
   - arr[3,2,1] <= 3528.26760379047
 travel_time_from2to3_drone3_trip0: 3600 x[2,3,3,0] + dep[2,3,0]
   - arr[3,3,0] <= 3528.26760379047
 travel_time_from2to3_drone3_trip1: 3600 x[2,3,3,1] + dep[2,3,1]
   - arr[3,3,1] <= 3528.26760379047
 travel_time_from2to3_drone4_trip0: 3600 x[2,3,4,0] + dep[2,4,0]
   - arr[3,4,0] <= 3528.26760379047
 travel_time_from2to3_drone4_trip1: 3600 x[2,3,4,1] + dep[2,4,1]
   - arr[3,4,1] <= 3528.26760379047
 travel_time_from2to4_drone0_trip0: 3600 x[2,4,0,0] + dep[2,0,0]
   - arr[4,0,0] <= 3539.488288837398
 travel_time_from2to4_drone0_trip1: 3600 x[2,4,0,1] + dep[2,0,1]
   - arr[4,0,1] <= 3539.488288837398
 travel_time_from2to4_drone1_trip0: 3600 x[2,4,1,0] + dep[2,1,0]
   - arr[4,1,0] <= 3539.488288837398
 travel_time_from2to4_drone1_trip1: 3600 x[2,4,1,1] + dep[2,1,1]
   - arr[4,1,1] <= 3539.488288837398
 travel_time_from2to4_drone2_trip0: 3600 x[2,4,2,0] + dep[2,2,0]
   - arr[4,2,0] <= 3539.488288837398
 travel_time_from2to4_drone2_trip1: 3600 x[2,4,2,1] + dep[2,2,1]
   - arr[4,2,1] <= 3539.488288837398
 travel_time_from2to4_drone3_trip0: 3600 x[2,4,3,0] + dep[2,3,0]
   - arr[4,3,0] <= 3539.488288837398
 travel_time_from2to4_drone3_trip1: 3600 x[2,4,3,1] + dep[2,3,1]
   - arr[4,3,1] <= 3539.488288837398
 travel_time_from2to4_drone4_trip0: 3600 x[2,4,4,0] + dep[2,4,0]
   - arr[4,4,0] <= 3539.488288837398
 travel_time_from2to4_drone4_trip1: 3600 x[2,4,4,1] + dep[2,4,1]
   - arr[4,4,1] <= 3539.488288837398
 travel_time_from2to5_drone0_trip0: 3600 x[2,5,0,0] + dep[2,0,0]
   - arr[5,0,0] <= 3455.106122413686
 travel_time_from2to5_drone0_trip1: 3600 x[2,5,0,1] + dep[2,0,1]
   - arr[5,0,1] <= 3455.106122413686
 travel_time_from2to5_drone1_trip0: 3600 x[2,5,1,0] + dep[2,1,0]
   - arr[5,1,0] <= 3455.106122413686
 travel_time_from2to5_drone1_trip1: 3600 x[2,5,1,1] + dep[2,1,1]
   - arr[5,1,1] <= 3455.106122413686
 travel_time_from2to5_drone2_trip0: 3600 x[2,5,2,0] + dep[2,2,0]
   - arr[5,2,0] <= 3455.106122413686
 travel_time_from2to5_drone2_trip1: 3600 x[2,5,2,1] + dep[2,2,1]
   - arr[5,2,1] <= 3455.106122413686
 travel_time_from2to5_drone3_trip0: 3600 x[2,5,3,0] + dep[2,3,0]
   - arr[5,3,0] <= 3455.106122413686
 travel_time_from2to5_drone3_trip1: 3600 x[2,5,3,1] + dep[2,3,1]
   - arr[5,3,1] <= 3455.106122413686
 travel_time_from2to5_drone4_trip0: 3600 x[2,5,4,0] + dep[2,4,0]
   - arr[5,4,0] <= 3455.106122413686
 travel_time_from2to5_drone4_trip1: 3600 x[2,5,4,1] + dep[2,4,1]
   - arr[5,4,1] <= 3455.106122413686
 travel_time_from2to6_drone0_trip0: 3600 x[2,6,0,0] + dep[2,0,0]
   - arr[6,0,0] <= 3508.56318476265
 travel_time_from2to6_drone0_trip1: 3600 x[2,6,0,1] + dep[2,0,1]
   - arr[6,0,1] <= 3508.56318476265
 travel_time_from2to6_drone1_trip0: 3600 x[2,6,1,0] + dep[2,1,0]
   - arr[6,1,0] <= 3508.56318476265
 travel_time_from2to6_drone1_trip1: 3600 x[2,6,1,1] + dep[2,1,1]
   - arr[6,1,1] <= 3508.56318476265
 travel_time_from2to6_drone2_trip0: 3600 x[2,6,2,0] + dep[2,2,0]
   - arr[6,2,0] <= 3508.56318476265
 travel_time_from2to6_drone2_trip1: 3600 x[2,6,2,1] + dep[2,2,1]
   - arr[6,2,1] <= 3508.56318476265
 travel_time_from2to6_drone3_trip0: 3600 x[2,6,3,0] + dep[2,3,0]
   - arr[6,3,0] <= 3508.56318476265
 travel_time_from2to6_drone3_trip1: 3600 x[2,6,3,1] + dep[2,3,1]
   - arr[6,3,1] <= 3508.56318476265
 travel_time_from2to6_drone4_trip0: 3600 x[2,6,4,0] + dep[2,4,0]
   - arr[6,4,0] <= 3508.56318476265
 travel_time_from2to6_drone4_trip1: 3600 x[2,6,4,1] + dep[2,4,1]
   - arr[6,4,1] <= 3508.56318476265
 travel_time_from3to0_drone0_trip0: 3600 x[3,0,0,0] - arr[0,0,0]
   + dep[3,0,0] <= 3541.263127204569
 travel_time_from3to0_drone0_trip1: 3600 x[3,0,0,1] - arr[0,0,1]
   + dep[3,0,1] <= 3541.263127204569
 travel_time_from3to0_drone1_trip0: 3600 x[3,0,1,0] - arr[0,1,0]
   + dep[3,1,0] <= 3541.263127204569
 travel_time_from3to0_drone1_trip1: 3600 x[3,0,1,1] - arr[0,1,1]
   + dep[3,1,1] <= 3541.263127204569
 travel_time_from3to0_drone2_trip0: 3600 x[3,0,2,0] - arr[0,2,0]
   + dep[3,2,0] <= 3541.263127204569
 travel_time_from3to0_drone2_trip1: 3600 x[3,0,2,1] - arr[0,2,1]
   + dep[3,2,1] <= 3541.263127204569
 travel_time_from3to0_drone3_trip0: 3600 x[3,0,3,0] - arr[0,3,0]
   + dep[3,3,0] <= 3541.263127204569
 travel_time_from3to0_drone3_trip1: 3600 x[3,0,3,1] - arr[0,3,1]
   + dep[3,3,1] <= 3541.263127204569
 travel_time_from3to0_drone4_trip0: 3600 x[3,0,4,0] - arr[0,4,0]
   + dep[3,4,0] <= 3541.263127204569
 travel_time_from3to0_drone4_trip1: 3600 x[3,0,4,1] - arr[0,4,1]
   + dep[3,4,1] <= 3541.263127204569
 travel_time_from3to1_drone0_trip0: 3600 x[3,1,0,0] - arr[1,0,0]
   + dep[3,0,0] <= 3507.927887506031
 travel_time_from3to1_drone0_trip1: 3600 x[3,1,0,1] - arr[1,0,1]
   + dep[3,0,1] <= 3507.927887506031
 travel_time_from3to1_drone1_trip0: 3600 x[3,1,1,0] - arr[1,1,0]
   + dep[3,1,0] <= 3507.927887506031
 travel_time_from3to1_drone1_trip1: 3600 x[3,1,1,1] - arr[1,1,1]
   + dep[3,1,1] <= 3507.927887506031
 travel_time_from3to1_drone2_trip0: 3600 x[3,1,2,0] - arr[1,2,0]
   + dep[3,2,0] <= 3507.927887506031
 travel_time_from3to1_drone2_trip1: 3600 x[3,1,2,1] - arr[1,2,1]
   + dep[3,2,1] <= 3507.927887506031
 travel_time_from3to1_drone3_trip0: 3600 x[3,1,3,0] - arr[1,3,0]
   + dep[3,3,0] <= 3507.927887506031
 travel_time_from3to1_drone3_trip1: 3600 x[3,1,3,1] - arr[1,3,1]
   + dep[3,3,1] <= 3507.927887506031
 travel_time_from3to1_drone4_trip0: 3600 x[3,1,4,0] - arr[1,4,0]
   + dep[3,4,0] <= 3507.927887506031
 travel_time_from3to1_drone4_trip1: 3600 x[3,1,4,1] - arr[1,4,1]
   + dep[3,4,1] <= 3507.927887506031
 travel_time_from3to2_drone0_trip0: 3600 x[3,2,0,0] - arr[2,0,0]
   + dep[3,0,0] <= 3528.26760379047
 travel_time_from3to2_drone0_trip1: 3600 x[3,2,0,1] - arr[2,0,1]
   + dep[3,0,1] <= 3528.26760379047
 travel_time_from3to2_drone1_trip0: 3600 x[3,2,1,0] - arr[2,1,0]
   + dep[3,1,0] <= 3528.26760379047
 travel_time_from3to2_drone1_trip1: 3600 x[3,2,1,1] - arr[2,1,1]
   + dep[3,1,1] <= 3528.26760379047
 travel_time_from3to2_drone2_trip0: 3600 x[3,2,2,0] - arr[2,2,0]
   + dep[3,2,0] <= 3528.26760379047
 travel_time_from3to2_drone2_trip1: 3600 x[3,2,2,1] - arr[2,2,1]
   + dep[3,2,1] <= 3528.26760379047
 travel_time_from3to2_drone3_trip0: 3600 x[3,2,3,0] - arr[2,3,0]
   + dep[3,3,0] <= 3528.26760379047
 travel_time_from3to2_drone3_trip1: 3600 x[3,2,3,1] - arr[2,3,1]
   + dep[3,3,1] <= 3528.26760379047
 travel_time_from3to2_drone4_trip0: 3600 x[3,2,4,0] - arr[2,4,0]
   + dep[3,4,0] <= 3528.26760379047
 travel_time_from3to2_drone4_trip1: 3600 x[3,2,4,1] - arr[2,4,1]
   + dep[3,4,1] <= 3528.26760379047
 travel_time_from3to4_drone0_trip0: 3600 x[3,4,0,0] + dep[3,0,0]
   - arr[4,0,0] <= 3503.008540356405
 travel_time_from3to4_drone0_trip1: 3600 x[3,4,0,1] + dep[3,0,1]
   - arr[4,0,1] <= 3503.008540356405
 travel_time_from3to4_drone1_trip0: 3600 x[3,4,1,0] + dep[3,1,0]
   - arr[4,1,0] <= 3503.008540356405
 travel_time_from3to4_drone1_trip1: 3600 x[3,4,1,1] + dep[3,1,1]
   - arr[4,1,1] <= 3503.008540356405
 travel_time_from3to4_drone2_trip0: 3600 x[3,4,2,0] + dep[3,2,0]
   - arr[4,2,0] <= 3503.008540356405
 travel_time_from3to4_drone2_trip1: 3600 x[3,4,2,1] + dep[3,2,1]
   - arr[4,2,1] <= 3503.008540356405
 travel_time_from3to4_drone3_trip0: 3600 x[3,4,3,0] + dep[3,3,0]
   - arr[4,3,0] <= 3503.008540356405
 travel_time_from3to4_drone3_trip1: 3600 x[3,4,3,1] + dep[3,3,1]
   - arr[4,3,1] <= 3503.008540356405
 travel_time_from3to4_drone4_trip0: 3600 x[3,4,4,0] + dep[3,4,0]
   - arr[4,4,0] <= 3503.008540356405
 travel_time_from3to4_drone4_trip1: 3600 x[3,4,4,1] + dep[3,4,1]
   - arr[4,4,1] <= 3503.008540356405
 travel_time_from3to5_drone0_trip0: 3600 x[3,5,0,0] + dep[3,0,0]
   - arr[5,0,0] <= 3515.71892190814
 travel_time_from3to5_drone0_trip1: 3600 x[3,5,0,1] + dep[3,0,1]
   - arr[5,0,1] <= 3515.71892190814
 travel_time_from3to5_drone1_trip0: 3600 x[3,5,1,0] + dep[3,1,0]
   - arr[5,1,0] <= 3515.71892190814
 travel_time_from3to5_drone1_trip1: 3600 x[3,5,1,1] + dep[3,1,1]
   - arr[5,1,1] <= 3515.71892190814
 travel_time_from3to5_drone2_trip0: 3600 x[3,5,2,0] + dep[3,2,0]
   - arr[5,2,0] <= 3515.71892190814
 travel_time_from3to5_drone2_trip1: 3600 x[3,5,2,1] + dep[3,2,1]
   - arr[5,2,1] <= 3515.71892190814
 travel_time_from3to5_drone3_trip0: 3600 x[3,5,3,0] + dep[3,3,0]
   - arr[5,3,0] <= 3515.71892190814
 travel_time_from3to5_drone3_trip1: 3600 x[3,5,3,1] + dep[3,3,1]
   - arr[5,3,1] <= 3515.71892190814
 travel_time_from3to5_drone4_trip0: 3600 x[3,5,4,0] + dep[3,4,0]
   - arr[5,4,0] <= 3515.71892190814
 travel_time_from3to5_drone4_trip1: 3600 x[3,5,4,1] + dep[3,4,1]
   - arr[5,4,1] <= 3515.71892190814
 travel_time_from3to6_drone0_trip0: 3600 x[3,6,0,0] + dep[3,0,0]
   - arr[6,0,0] <= 3544.726997902158
 travel_time_from3to6_drone0_trip1: 3600 x[3,6,0,1] + dep[3,0,1]
   - arr[6,0,1] <= 3544.726997902158
 travel_time_from3to6_drone1_trip0: 3600 x[3,6,1,0] + dep[3,1,0]
   - arr[6,1,0] <= 3544.726997902158
 travel_time_from3to6_drone1_trip1: 3600 x[3,6,1,1] + dep[3,1,1]
   - arr[6,1,1] <= 3544.726997902158
 travel_time_from3to6_drone2_trip0: 3600 x[3,6,2,0] + dep[3,2,0]
   - arr[6,2,0] <= 3544.726997902158
 travel_time_from3to6_drone2_trip1: 3600 x[3,6,2,1] + dep[3,2,1]
   - arr[6,2,1] <= 3544.726997902158
 travel_time_from3to6_drone3_trip0: 3600 x[3,6,3,0] + dep[3,3,0]
   - arr[6,3,0] <= 3544.726997902158
 travel_time_from3to6_drone3_trip1: 3600 x[3,6,3,1] + dep[3,3,1]
   - arr[6,3,1] <= 3544.726997902158
 travel_time_from3to6_drone4_trip0: 3600 x[3,6,4,0] + dep[3,4,0]
   - arr[6,4,0] <= 3544.726997902158
 travel_time_from3to6_drone4_trip1: 3600 x[3,6,4,1] + dep[3,4,1]
   - arr[6,4,1] <= 3544.726997902158
 travel_time_from4to0_drone0_trip0: 3600 x[4,0,0,0] - arr[0,0,0]
   + dep[4,0,0] <= 3539.79410277222
 travel_time_from4to0_drone0_trip1: 3600 x[4,0,0,1] - arr[0,0,1]
   + dep[4,0,1] <= 3539.79410277222
 travel_time_from4to0_drone1_trip0: 3600 x[4,0,1,0] - arr[0,1,0]
   + dep[4,1,0] <= 3539.79410277222
 travel_time_from4to0_drone1_trip1: 3600 x[4,0,1,1] - arr[0,1,1]
   + dep[4,1,1] <= 3539.79410277222
 travel_time_from4to0_drone2_trip0: 3600 x[4,0,2,0] - arr[0,2,0]
   + dep[4,2,0] <= 3539.79410277222
 travel_time_from4to0_drone2_trip1: 3600 x[4,0,2,1] - arr[0,2,1]
   + dep[4,2,1] <= 3539.79410277222
 travel_time_from4to0_drone3_trip0: 3600 x[4,0,3,0] - arr[0,3,0]
   + dep[4,3,0] <= 3539.79410277222
 travel_time_from4to0_drone3_trip1: 3600 x[4,0,3,1] - arr[0,3,1]
   + dep[4,3,1] <= 3539.79410277222
 travel_time_from4to0_drone4_trip0: 3600 x[4,0,4,0] - arr[0,4,0]
   + dep[4,4,0] <= 3539.79410277222
 travel_time_from4to0_drone4_trip1: 3600 x[4,0,4,1] - arr[0,4,1]
   + dep[4,4,1] <= 3539.79410277222
 travel_time_from4to1_drone0_trip0: 3600 x[4,1,0,0] - arr[1,0,0]
   + dep[4,0,0] <= 3573.277657470349
 travel_time_from4to1_drone0_trip1: 3600 x[4,1,0,1] - arr[1,0,1]
   + dep[4,0,1] <= 3573.277657470349
 travel_time_from4to1_drone1_trip0: 3600 x[4,1,1,0] - arr[1,1,0]
   + dep[4,1,0] <= 3573.277657470349
 travel_time_from4to1_drone1_trip1: 3600 x[4,1,1,1] - arr[1,1,1]
   + dep[4,1,1] <= 3573.277657470349
 travel_time_from4to1_drone2_trip0: 3600 x[4,1,2,0] - arr[1,2,0]
   + dep[4,2,0] <= 3573.277657470349
 travel_time_from4to1_drone2_trip1: 3600 x[4,1,2,1] - arr[1,2,1]
   + dep[4,2,1] <= 3573.277657470349
 travel_time_from4to1_drone3_trip0: 3600 x[4,1,3,0] - arr[1,3,0]
   + dep[4,3,0] <= 3573.277657470349
 travel_time_from4to1_drone3_trip1: 3600 x[4,1,3,1] - arr[1,3,1]
   + dep[4,3,1] <= 3573.277657470349
 travel_time_from4to1_drone4_trip0: 3600 x[4,1,4,0] - arr[1,4,0]
   + dep[4,4,0] <= 3573.277657470349
 travel_time_from4to1_drone4_trip1: 3600 x[4,1,4,1] - arr[1,4,1]
   + dep[4,4,1] <= 3573.277657470349
 travel_time_from4to2_drone0_trip0: 3600 x[4,2,0,0] - arr[2,0,0]
   + dep[4,0,0] <= 3539.488288837398
 travel_time_from4to2_drone0_trip1: 3600 x[4,2,0,1] - arr[2,0,1]
   + dep[4,0,1] <= 3539.488288837398
 travel_time_from4to2_drone1_trip0: 3600 x[4,2,1,0] - arr[2,1,0]
   + dep[4,1,0] <= 3539.488288837398
 travel_time_from4to2_drone1_trip1: 3600 x[4,2,1,1] - arr[2,1,1]
   + dep[4,1,1] <= 3539.488288837398
 travel_time_from4to2_drone2_trip0: 3600 x[4,2,2,0] - arr[2,2,0]
   + dep[4,2,0] <= 3539.488288837398
 travel_time_from4to2_drone2_trip1: 3600 x[4,2,2,1] - arr[2,2,1]
   + dep[4,2,1] <= 3539.488288837398
 travel_time_from4to2_drone3_trip0: 3600 x[4,2,3,0] - arr[2,3,0]
   + dep[4,3,0] <= 3539.488288837398
 travel_time_from4to2_drone3_trip1: 3600 x[4,2,3,1] - arr[2,3,1]
   + dep[4,3,1] <= 3539.488288837398
 travel_time_from4to2_drone4_trip0: 3600 x[4,2,4,0] - arr[2,4,0]
   + dep[4,4,0] <= 3539.488288837398
 travel_time_from4to2_drone4_trip1: 3600 x[4,2,4,1] - arr[2,4,1]
   + dep[4,4,1] <= 3539.488288837398
 travel_time_from4to3_drone0_trip0: 3600 x[4,3,0,0] - arr[3,0,0]
   + dep[4,0,0] <= 3503.008540356405
 travel_time_from4to3_drone0_trip1: 3600 x[4,3,0,1] - arr[3,0,1]
   + dep[4,0,1] <= 3503.008540356405
 travel_time_from4to3_drone1_trip0: 3600 x[4,3,1,0] - arr[3,1,0]
   + dep[4,1,0] <= 3503.008540356405
 travel_time_from4to3_drone1_trip1: 3600 x[4,3,1,1] - arr[3,1,1]
   + dep[4,1,1] <= 3503.008540356405
 travel_time_from4to3_drone2_trip0: 3600 x[4,3,2,0] - arr[3,2,0]
   + dep[4,2,0] <= 3503.008540356405
 travel_time_from4to3_drone2_trip1: 3600 x[4,3,2,1] - arr[3,2,1]
   + dep[4,2,1] <= 3503.008540356405
 travel_time_from4to3_drone3_trip0: 3600 x[4,3,3,0] - arr[3,3,0]
   + dep[4,3,0] <= 3503.008540356405
 travel_time_from4to3_drone3_trip1: 3600 x[4,3,3,1] - arr[3,3,1]
   + dep[4,3,1] <= 3503.008540356405
 travel_time_from4to3_drone4_trip0: 3600 x[4,3,4,0] - arr[3,4,0]
   + dep[4,4,0] <= 3503.008540356405
 travel_time_from4to3_drone4_trip1: 3600 x[4,3,4,1] - arr[3,4,1]
   + dep[4,4,1] <= 3503.008540356405
 travel_time_from4to5_drone0_trip0: 3600 x[4,5,0,0] + dep[4,0,0]
   - arr[5,0,0] <= 3463.108782812808
 travel_time_from4to5_drone0_trip1: 3600 x[4,5,0,1] + dep[4,0,1]
   - arr[5,0,1] <= 3463.108782812808
 travel_time_from4to5_drone1_trip0: 3600 x[4,5,1,0] + dep[4,1,0]
   - arr[5,1,0] <= 3463.108782812808
 travel_time_from4to5_drone1_trip1: 3600 x[4,5,1,1] + dep[4,1,1]
   - arr[5,1,1] <= 3463.108782812808
 travel_time_from4to5_drone2_trip0: 3600 x[4,5,2,0] + dep[4,2,0]
   - arr[5,2,0] <= 3463.108782812808
 travel_time_from4to5_drone2_trip1: 3600 x[4,5,2,1] + dep[4,2,1]
   - arr[5,2,1] <= 3463.108782812808
 travel_time_from4to5_drone3_trip0: 3600 x[4,5,3,0] + dep[4,3,0]
   - arr[5,3,0] <= 3463.108782812808
 travel_time_from4to5_drone3_trip1: 3600 x[4,5,3,1] + dep[4,3,1]
   - arr[5,3,1] <= 3463.108782812808
 travel_time_from4to5_drone4_trip0: 3600 x[4,5,4,0] + dep[4,4,0]
   - arr[5,4,0] <= 3463.108782812808
 travel_time_from4to5_drone4_trip1: 3600 x[4,5,4,1] + dep[4,4,1]
   - arr[5,4,1] <= 3463.108782812808
 travel_time_from4to6_drone0_trip0: 3600 x[4,6,0,0] + dep[4,0,0]
   - arr[6,0,0] <= 3461.061896652813
 travel_time_from4to6_drone0_trip1: 3600 x[4,6,0,1] + dep[4,0,1]
   - arr[6,0,1] <= 3461.061896652813
 travel_time_from4to6_drone1_trip0: 3600 x[4,6,1,0] + dep[4,1,0]
   - arr[6,1,0] <= 3461.061896652813
 travel_time_from4to6_drone1_trip1: 3600 x[4,6,1,1] + dep[4,1,1]
   - arr[6,1,1] <= 3461.061896652813
 travel_time_from4to6_drone2_trip0: 3600 x[4,6,2,0] + dep[4,2,0]
   - arr[6,2,0] <= 3461.061896652813
 travel_time_from4to6_drone2_trip1: 3600 x[4,6,2,1] + dep[4,2,1]
   - arr[6,2,1] <= 3461.061896652813
 travel_time_from4to6_drone3_trip0: 3600 x[4,6,3,0] + dep[4,3,0]
   - arr[6,3,0] <= 3461.061896652813
 travel_time_from4to6_drone3_trip1: 3600 x[4,6,3,1] + dep[4,3,1]
   - arr[6,3,1] <= 3461.061896652813
 travel_time_from4to6_drone4_trip0: 3600 x[4,6,4,0] + dep[4,4,0]
   - arr[6,4,0] <= 3461.061896652813
 travel_time_from4to6_drone4_trip1: 3600 x[4,6,4,1] + dep[4,4,1]
   - arr[6,4,1] <= 3461.061896652813
 travel_time_from5to0_drone0_trip0: 3600 x[5,0,0,0] - arr[0,0,0]
   + dep[5,0,0] <= 3523.194100412758
 travel_time_from5to0_drone0_trip1: 3600 x[5,0,0,1] - arr[0,0,1]
   + dep[5,0,1] <= 3523.194100412758
 travel_time_from5to0_drone1_trip0: 3600 x[5,0,1,0] - arr[0,1,0]
   + dep[5,1,0] <= 3523.194100412758
 travel_time_from5to0_drone1_trip1: 3600 x[5,0,1,1] - arr[0,1,1]
   + dep[5,1,1] <= 3523.194100412758
 travel_time_from5to0_drone2_trip0: 3600 x[5,0,2,0] - arr[0,2,0]
   + dep[5,2,0] <= 3523.194100412758
 travel_time_from5to0_drone2_trip1: 3600 x[5,0,2,1] - arr[0,2,1]
   + dep[5,2,1] <= 3523.194100412758
 travel_time_from5to0_drone3_trip0: 3600 x[5,0,3,0] - arr[0,3,0]
   + dep[5,3,0] <= 3523.194100412758
 travel_time_from5to0_drone3_trip1: 3600 x[5,0,3,1] - arr[0,3,1]
   + dep[5,3,1] <= 3523.194100412758
 travel_time_from5to0_drone4_trip0: 3600 x[5,0,4,0] - arr[0,4,0]
   + dep[5,4,0] <= 3523.194100412758
 travel_time_from5to0_drone4_trip1: 3600 x[5,0,4,1] - arr[0,4,1]
   + dep[5,4,1] <= 3523.194100412758
 travel_time_from5to1_drone0_trip0: 3600 x[5,1,0,0] - arr[1,0,0]
   + dep[5,0,0] <= 3484.317389583469
 travel_time_from5to1_drone0_trip1: 3600 x[5,1,0,1] - arr[1,0,1]
   + dep[5,0,1] <= 3484.317389583469
 travel_time_from5to1_drone1_trip0: 3600 x[5,1,1,0] - arr[1,1,0]
   + dep[5,1,0] <= 3484.317389583469
 travel_time_from5to1_drone1_trip1: 3600 x[5,1,1,1] - arr[1,1,1]
   + dep[5,1,1] <= 3484.317389583469
 travel_time_from5to1_drone2_trip0: 3600 x[5,1,2,0] - arr[1,2,0]
   + dep[5,2,0] <= 3484.317389583469
 travel_time_from5to1_drone2_trip1: 3600 x[5,1,2,1] - arr[1,2,1]
   + dep[5,2,1] <= 3484.317389583469
 travel_time_from5to1_drone3_trip0: 3600 x[5,1,3,0] - arr[1,3,0]
   + dep[5,3,0] <= 3484.317389583469
 travel_time_from5to1_drone3_trip1: 3600 x[5,1,3,1] - arr[1,3,1]
   + dep[5,3,1] <= 3484.317389583469
 travel_time_from5to1_drone4_trip0: 3600 x[5,1,4,0] - arr[1,4,0]
   + dep[5,4,0] <= 3484.317389583469
 travel_time_from5to1_drone4_trip1: 3600 x[5,1,4,1] - arr[1,4,1]
   + dep[5,4,1] <= 3484.317389583469
 travel_time_from5to2_drone0_trip0: 3600 x[5,2,0,0] - arr[2,0,0]
   + dep[5,0,0] <= 3455.106122413686
 travel_time_from5to2_drone0_trip1: 3600 x[5,2,0,1] - arr[2,0,1]
   + dep[5,0,1] <= 3455.106122413686
 travel_time_from5to2_drone1_trip0: 3600 x[5,2,1,0] - arr[2,1,0]
   + dep[5,1,0] <= 3455.106122413686
 travel_time_from5to2_drone1_trip1: 3600 x[5,2,1,1] - arr[2,1,1]
   + dep[5,1,1] <= 3455.106122413686
 travel_time_from5to2_drone2_trip0: 3600 x[5,2,2,0] - arr[2,2,0]
   + dep[5,2,0] <= 3455.106122413686
 travel_time_from5to2_drone2_trip1: 3600 x[5,2,2,1] - arr[2,2,1]
   + dep[5,2,1] <= 3455.106122413686
 travel_time_from5to2_drone3_trip0: 3600 x[5,2,3,0] - arr[2,3,0]
   + dep[5,3,0] <= 3455.106122413686
 travel_time_from5to2_drone3_trip1: 3600 x[5,2,3,1] - arr[2,3,1]
   + dep[5,3,1] <= 3455.106122413686
 travel_time_from5to2_drone4_trip0: 3600 x[5,2,4,0] - arr[2,4,0]
   + dep[5,4,0] <= 3455.106122413686
 travel_time_from5to2_drone4_trip1: 3600 x[5,2,4,1] - arr[2,4,1]
   + dep[5,4,1] <= 3455.106122413686
 travel_time_from5to3_drone0_trip0: 3600 x[5,3,0,0] - arr[3,0,0]
   + dep[5,0,0] <= 3515.71892190814
 travel_time_from5to3_drone0_trip1: 3600 x[5,3,0,1] - arr[3,0,1]
   + dep[5,0,1] <= 3515.71892190814
 travel_time_from5to3_drone1_trip0: 3600 x[5,3,1,0] - arr[3,1,0]
   + dep[5,1,0] <= 3515.71892190814
 travel_time_from5to3_drone1_trip1: 3600 x[5,3,1,1] - arr[3,1,1]
   + dep[5,1,1] <= 3515.71892190814
 travel_time_from5to3_drone2_trip0: 3600 x[5,3,2,0] - arr[3,2,0]
   + dep[5,2,0] <= 3515.71892190814
 travel_time_from5to3_drone2_trip1: 3600 x[5,3,2,1] - arr[3,2,1]
   + dep[5,2,1] <= 3515.71892190814
 travel_time_from5to3_drone3_trip0: 3600 x[5,3,3,0] - arr[3,3,0]
   + dep[5,3,0] <= 3515.71892190814
 travel_time_from5to3_drone3_trip1: 3600 x[5,3,3,1] - arr[3,3,1]
   + dep[5,3,1] <= 3515.71892190814
 travel_time_from5to3_drone4_trip0: 3600 x[5,3,4,0] - arr[3,4,0]
   + dep[5,4,0] <= 3515.71892190814
 travel_time_from5to3_drone4_trip1: 3600 x[5,3,4,1] - arr[3,4,1]
   + dep[5,4,1] <= 3515.71892190814
 travel_time_from5to4_drone0_trip0: 3600 x[5,4,0,0] - arr[4,0,0]
   + dep[5,0,0] <= 3463.108782812808
 travel_time_from5to4_drone0_trip1: 3600 x[5,4,0,1] - arr[4,0,1]
   + dep[5,0,1] <= 3463.108782812808
 travel_time_from5to4_drone1_trip0: 3600 x[5,4,1,0] - arr[4,1,0]
   + dep[5,1,0] <= 3463.108782812808
 travel_time_from5to4_drone1_trip1: 3600 x[5,4,1,1] - arr[4,1,1]
   + dep[5,1,1] <= 3463.108782812808
 travel_time_from5to4_drone2_trip0: 3600 x[5,4,2,0] - arr[4,2,0]
   + dep[5,2,0] <= 3463.108782812808
 travel_time_from5to4_drone2_trip1: 3600 x[5,4,2,1] - arr[4,2,1]
   + dep[5,2,1] <= 3463.108782812808
 travel_time_from5to4_drone3_trip0: 3600 x[5,4,3,0] - arr[4,3,0]
   + dep[5,3,0] <= 3463.108782812808
 travel_time_from5to4_drone3_trip1: 3600 x[5,4,3,1] - arr[4,3,1]
   + dep[5,3,1] <= 3463.108782812808
 travel_time_from5to4_drone4_trip0: 3600 x[5,4,4,0] - arr[4,4,0]
   + dep[5,4,0] <= 3463.108782812808
 travel_time_from5to4_drone4_trip1: 3600 x[5,4,4,1] - arr[4,4,1]
   + dep[5,4,1] <= 3463.108782812808
 travel_time_from5to6_drone0_trip0: 3600 x[5,6,0,0] + dep[5,0,0]
   - arr[6,0,0] <= 3471.723050895672
 travel_time_from5to6_drone0_trip1: 3600 x[5,6,0,1] + dep[5,0,1]
   - arr[6,0,1] <= 3471.723050895672
 travel_time_from5to6_drone1_trip0: 3600 x[5,6,1,0] + dep[5,1,0]
   - arr[6,1,0] <= 3471.723050895672
 travel_time_from5to6_drone1_trip1: 3600 x[5,6,1,1] + dep[5,1,1]
   - arr[6,1,1] <= 3471.723050895672
 travel_time_from5to6_drone2_trip0: 3600 x[5,6,2,0] + dep[5,2,0]
   - arr[6,2,0] <= 3471.723050895672
 travel_time_from5to6_drone2_trip1: 3600 x[5,6,2,1] + dep[5,2,1]
   - arr[6,2,1] <= 3471.723050895672
 travel_time_from5to6_drone3_trip0: 3600 x[5,6,3,0] + dep[5,3,0]
   - arr[6,3,0] <= 3471.723050895672
 travel_time_from5to6_drone3_trip1: 3600 x[5,6,3,1] + dep[5,3,1]
   - arr[6,3,1] <= 3471.723050895672
 travel_time_from5to6_drone4_trip0: 3600 x[5,6,4,0] + dep[5,4,0]
   - arr[6,4,0] <= 3471.723050895672
 travel_time_from5to6_drone4_trip1: 3600 x[5,6,4,1] + dep[5,4,1]
   - arr[6,4,1] <= 3471.723050895672
 travel_time_from6to0_drone0_trip0: 3600 x[6,0,0,0] - arr[0,0,0]
   + dep[6,0,0] <= 3486.898082960795
 travel_time_from6to0_drone0_trip1: 3600 x[6,0,0,1] - arr[0,0,1]
   + dep[6,0,1] <= 3486.898082960795
 travel_time_from6to0_drone1_trip0: 3600 x[6,0,1,0] - arr[0,1,0]
   + dep[6,1,0] <= 3486.898082960795
 travel_time_from6to0_drone1_trip1: 3600 x[6,0,1,1] - arr[0,1,1]
   + dep[6,1,1] <= 3486.898082960795
 travel_time_from6to0_drone2_trip0: 3600 x[6,0,2,0] - arr[0,2,0]
   + dep[6,2,0] <= 3486.898082960795
 travel_time_from6to0_drone2_trip1: 3600 x[6,0,2,1] - arr[0,2,1]
   + dep[6,2,1] <= 3486.898082960795
 travel_time_from6to0_drone3_trip0: 3600 x[6,0,3,0] - arr[0,3,0]
   + dep[6,3,0] <= 3486.898082960795
 travel_time_from6to0_drone3_trip1: 3600 x[6,0,3,1] - arr[0,3,1]
   + dep[6,3,1] <= 3486.898082960795
 travel_time_from6to0_drone4_trip0: 3600 x[6,0,4,0] - arr[0,4,0]
   + dep[6,4,0] <= 3486.898082960795
 travel_time_from6to0_drone4_trip1: 3600 x[6,0,4,1] - arr[0,4,1]
   + dep[6,4,1] <= 3486.898082960795
 travel_time_from6to1_drone0_trip0: 3600 x[6,1,0,0] - arr[1,0,0]
   + dep[6,0,0] <= 3458.815710587392
 travel_time_from6to1_drone0_trip1: 3600 x[6,1,0,1] - arr[1,0,1]
   + dep[6,0,1] <= 3458.815710587392
 travel_time_from6to1_drone1_trip0: 3600 x[6,1,1,0] - arr[1,1,0]
   + dep[6,1,0] <= 3458.815710587392
 travel_time_from6to1_drone1_trip1: 3600 x[6,1,1,1] - arr[1,1,1]
   + dep[6,1,1] <= 3458.815710587392
 travel_time_from6to1_drone2_trip0: 3600 x[6,1,2,0] - arr[1,2,0]
   + dep[6,2,0] <= 3458.815710587392
 travel_time_from6to1_drone2_trip1: 3600 x[6,1,2,1] - arr[1,2,1]
   + dep[6,2,1] <= 3458.815710587392
 travel_time_from6to1_drone3_trip0: 3600 x[6,1,3,0] - arr[1,3,0]
   + dep[6,3,0] <= 3458.815710587392
 travel_time_from6to1_drone3_trip1: 3600 x[6,1,3,1] - arr[1,3,1]
   + dep[6,3,1] <= 3458.815710587392
 travel_time_from6to1_drone4_trip0: 3600 x[6,1,4,0] - arr[1,4,0]
   + dep[6,4,0] <= 3458.815710587392
 travel_time_from6to1_drone4_trip1: 3600 x[6,1,4,1] - arr[1,4,1]
   + dep[6,4,1] <= 3458.815710587392
 travel_time_from6to2_drone0_trip0: 3600 x[6,2,0,0] - arr[2,0,0]
   + dep[6,0,0] <= 3508.56318476265
 travel_time_from6to2_drone0_trip1: 3600 x[6,2,0,1] - arr[2,0,1]
   + dep[6,0,1] <= 3508.56318476265
 travel_time_from6to2_drone1_trip0: 3600 x[6,2,1,0] - arr[2,1,0]
   + dep[6,1,0] <= 3508.56318476265
 travel_time_from6to2_drone1_trip1: 3600 x[6,2,1,1] - arr[2,1,1]
   + dep[6,1,1] <= 3508.56318476265
 travel_time_from6to2_drone2_trip0: 3600 x[6,2,2,0] - arr[2,2,0]
   + dep[6,2,0] <= 3508.56318476265
 travel_time_from6to2_drone2_trip1: 3600 x[6,2,2,1] - arr[2,2,1]
   + dep[6,2,1] <= 3508.56318476265
 travel_time_from6to2_drone3_trip0: 3600 x[6,2,3,0] - arr[2,3,0]
   + dep[6,3,0] <= 3508.56318476265
 travel_time_from6to2_drone3_trip1: 3600 x[6,2,3,1] - arr[2,3,1]
   + dep[6,3,1] <= 3508.56318476265
 travel_time_from6to2_drone4_trip0: 3600 x[6,2,4,0] - arr[2,4,0]
   + dep[6,4,0] <= 3508.56318476265
 travel_time_from6to2_drone4_trip1: 3600 x[6,2,4,1] - arr[2,4,1]
   + dep[6,4,1] <= 3508.56318476265
 travel_time_from6to3_drone0_trip0: 3600 x[6,3,0,0] - arr[3,0,0]
   + dep[6,0,0] <= 3544.726997902158
 travel_time_from6to3_drone0_trip1: 3600 x[6,3,0,1] - arr[3,0,1]
   + dep[6,0,1] <= 3544.726997902158
 travel_time_from6to3_drone1_trip0: 3600 x[6,3,1,0] - arr[3,1,0]
   + dep[6,1,0] <= 3544.726997902158
 travel_time_from6to3_drone1_trip1: 3600 x[6,3,1,1] - arr[3,1,1]
   + dep[6,1,1] <= 3544.726997902158
 travel_time_from6to3_drone2_trip0: 3600 x[6,3,2,0] - arr[3,2,0]
   + dep[6,2,0] <= 3544.726997902158
 travel_time_from6to3_drone2_trip1: 3600 x[6,3,2,1] - arr[3,2,1]
   + dep[6,2,1] <= 3544.726997902158
 travel_time_from6to3_drone3_trip0: 3600 x[6,3,3,0] - arr[3,3,0]
   + dep[6,3,0] <= 3544.726997902158
 travel_time_from6to3_drone3_trip1: 3600 x[6,3,3,1] - arr[3,3,1]
   + dep[6,3,1] <= 3544.726997902158
 travel_time_from6to3_drone4_trip0: 3600 x[6,3,4,0] - arr[3,4,0]
   + dep[6,4,0] <= 3544.726997902158
 travel_time_from6to3_drone4_trip1: 3600 x[6,3,4,1] - arr[3,4,1]
   + dep[6,4,1] <= 3544.726997902158
 travel_time_from6to4_drone0_trip0: 3600 x[6,4,0,0] - arr[4,0,0]
   + dep[6,0,0] <= 3461.061896652813
 travel_time_from6to4_drone0_trip1: 3600 x[6,4,0,1] - arr[4,0,1]
   + dep[6,0,1] <= 3461.061896652813
 travel_time_from6to4_drone1_trip0: 3600 x[6,4,1,0] - arr[4,1,0]
   + dep[6,1,0] <= 3461.061896652813
 travel_time_from6to4_drone1_trip1: 3600 x[6,4,1,1] - arr[4,1,1]
   + dep[6,1,1] <= 3461.061896652813
 travel_time_from6to4_drone2_trip0: 3600 x[6,4,2,0] - arr[4,2,0]
   + dep[6,2,0] <= 3461.061896652813
 travel_time_from6to4_drone2_trip1: 3600 x[6,4,2,1] - arr[4,2,1]
   + dep[6,2,1] <= 3461.061896652813
 travel_time_from6to4_drone3_trip0: 3600 x[6,4,3,0] - arr[4,3,0]
   + dep[6,3,0] <= 3461.061896652813
 travel_time_from6to4_drone3_trip1: 3600 x[6,4,3,1] - arr[4,3,1]
   + dep[6,3,1] <= 3461.061896652813
 travel_time_from6to4_drone4_trip0: 3600 x[6,4,4,0] - arr[4,4,0]
   + dep[6,4,0] <= 3461.061896652813
 travel_time_from6to4_drone4_trip1: 3600 x[6,4,4,1] - arr[4,4,1]
   + dep[6,4,1] <= 3461.061896652813
 travel_time_from6to5_drone0_trip0: 3600 x[6,5,0,0] - arr[5,0,0]
   + dep[6,0,0] <= 3471.723050895672
 travel_time_from6to5_drone0_trip1: 3600 x[6,5,0,1] - arr[5,0,1]
   + dep[6,0,1] <= 3471.723050895672
 travel_time_from6to5_drone1_trip0: 3600 x[6,5,1,0] - arr[5,1,0]
   + dep[6,1,0] <= 3471.723050895672
 travel_time_from6to5_drone1_trip1: 3600 x[6,5,1,1] - arr[5,1,1]
   + dep[6,1,1] <= 3471.723050895672
 travel_time_from6to5_drone2_trip0: 3600 x[6,5,2,0] - arr[5,2,0]
   + dep[6,2,0] <= 3471.723050895672
 travel_time_from6to5_drone2_trip1: 3600 x[6,5,2,1] - arr[5,2,1]
   + dep[6,2,1] <= 3471.723050895672
 travel_time_from6to5_drone3_trip0: 3600 x[6,5,3,0] - arr[5,3,0]
   + dep[6,3,0] <= 3471.723050895672
 travel_time_from6to5_drone3_trip1: 3600 x[6,5,3,1] - arr[5,3,1]
   + dep[6,3,1] <= 3471.723050895672
 travel_time_from6to5_drone4_trip0: 3600 x[6,5,4,0] - arr[5,4,0]
   + dep[6,4,0] <= 3471.723050895672
 travel_time_from6to5_drone4_trip1: 3600 x[6,5,4,1] - arr[5,4,1]
   + dep[6,4,1] <= 3471.723050895672
 drop_time_node1_drone0_trip0: 10 p[1,0,0] + arr[1,0,0] - dep[1,0,0] = 0
 drop_time_node1_drone0_trip1: 10 p[1,0,1] + arr[1,0,1] - dep[1,0,1] = 0
 drop_time_node1_drone1_trip0: 10 p[1,1,0] + arr[1,1,0] - dep[1,1,0] = 0
 drop_time_node1_drone1_trip1: 10 p[1,1,1] + arr[1,1,1] - dep[1,1,1] = 0
 drop_time_node1_drone2_trip0: 10 p[1,2,0] + arr[1,2,0] - dep[1,2,0] = 0
 drop_time_node1_drone2_trip1: 10 p[1,2,1] + arr[1,2,1] - dep[1,2,1] = 0
 drop_time_node1_drone3_trip0: 10 p[1,3,0] + arr[1,3,0] - dep[1,3,0] = 0
 drop_time_node1_drone3_trip1: 10 p[1,3,1] + arr[1,3,1] - dep[1,3,1] = 0
 drop_time_node1_drone4_trip0: 10 p[1,4,0] + arr[1,4,0] - dep[1,4,0] = 0
 drop_time_node1_drone4_trip1: 10 p[1,4,1] + arr[1,4,1] - dep[1,4,1] = 0
 drop_time_node2_drone0_trip0: 10 p[2,0,0] + arr[2,0,0] - dep[2,0,0] = 0
 drop_time_node2_drone0_trip1: 10 p[2,0,1] + arr[2,0,1] - dep[2,0,1] = 0
 drop_time_node2_drone1_trip0: 10 p[2,1,0] + arr[2,1,0] - dep[2,1,0] = 0
 drop_time_node2_drone1_trip1: 10 p[2,1,1] + arr[2,1,1] - dep[2,1,1] = 0
 drop_time_node2_drone2_trip0: 10 p[2,2,0] + arr[2,2,0] - dep[2,2,0] = 0
 drop_time_node2_drone2_trip1: 10 p[2,2,1] + arr[2,2,1] - dep[2,2,1] = 0
 drop_time_node2_drone3_trip0: 10 p[2,3,0] + arr[2,3,0] - dep[2,3,0] = 0
 drop_time_node2_drone3_trip1: 10 p[2,3,1] + arr[2,3,1] - dep[2,3,1] = 0
 drop_time_node2_drone4_trip0: 10 p[2,4,0] + arr[2,4,0] - dep[2,4,0] = 0
 drop_time_node2_drone4_trip1: 10 p[2,4,1] + arr[2,4,1] - dep[2,4,1] = 0
 drop_time_node3_drone0_trip0: 10 p[3,0,0] + arr[3,0,0] - dep[3,0,0] = 0
 drop_time_node3_drone0_trip1: 10 p[3,0,1] + arr[3,0,1] - dep[3,0,1] = 0
 drop_time_node3_drone1_trip0: 10 p[3,1,0] + arr[3,1,0] - dep[3,1,0] = 0
 drop_time_node3_drone1_trip1: 10 p[3,1,1] + arr[3,1,1] - dep[3,1,1] = 0
 drop_time_node3_drone2_trip0: 10 p[3,2,0] + arr[3,2,0] - dep[3,2,0] = 0
 drop_time_node3_drone2_trip1: 10 p[3,2,1] + arr[3,2,1] - dep[3,2,1] = 0
 drop_time_node3_drone3_trip0: 10 p[3,3,0] + arr[3,3,0] - dep[3,3,0] = 0
 drop_time_node3_drone3_trip1: 10 p[3,3,1] + arr[3,3,1] - dep[3,3,1] = 0
 drop_time_node3_drone4_trip0: 10 p[3,4,0] + arr[3,4,0] - dep[3,4,0] = 0
 drop_time_node3_drone4_trip1: 10 p[3,4,1] + arr[3,4,1] - dep[3,4,1] = 0
 drop_time_node4_drone0_trip0: 10 p[4,0,0] + arr[4,0,0] - dep[4,0,0] = 0
 drop_time_node4_drone0_trip1: 10 p[4,0,1] + arr[4,0,1] - dep[4,0,1] = 0
 drop_time_node4_drone1_trip0: 10 p[4,1,0] + arr[4,1,0] - dep[4,1,0] = 0
 drop_time_node4_drone1_trip1: 10 p[4,1,1] + arr[4,1,1] - dep[4,1,1] = 0
 drop_time_node4_drone2_trip0: 10 p[4,2,0] + arr[4,2,0] - dep[4,2,0] = 0
 drop_time_node4_drone2_trip1: 10 p[4,2,1] + arr[4,2,1] - dep[4,2,1] = 0
 drop_time_node4_drone3_trip0: 10 p[4,3,0] + arr[4,3,0] - dep[4,3,0] = 0
 drop_time_node4_drone3_trip1: 10 p[4,3,1] + arr[4,3,1] - dep[4,3,1] = 0
 drop_time_node4_drone4_trip0: 10 p[4,4,0] + arr[4,4,0] - dep[4,4,0] = 0
 drop_time_node4_drone4_trip1: 10 p[4,4,1] + arr[4,4,1] - dep[4,4,1] = 0
 drop_time_node5_drone0_trip0: 10 p[5,0,0] + arr[5,0,0] - dep[5,0,0] = 0
 drop_time_node5_drone0_trip1: 10 p[5,0,1] + arr[5,0,1] - dep[5,0,1] = 0
 drop_time_node5_drone1_trip0: 10 p[5,1,0] + arr[5,1,0] - dep[5,1,0] = 0
 drop_time_node5_drone1_trip1: 10 p[5,1,1] + arr[5,1,1] - dep[5,1,1] = 0
 drop_time_node5_drone2_trip0: 10 p[5,2,0] + arr[5,2,0] - dep[5,2,0] = 0
 drop_time_node5_drone2_trip1: 10 p[5,2,1] + arr[5,2,1] - dep[5,2,1] = 0
 drop_time_node5_drone3_trip0: 10 p[5,3,0] + arr[5,3,0] - dep[5,3,0] = 0
 drop_time_node5_drone3_trip1: 10 p[5,3,1] + arr[5,3,1] - dep[5,3,1] = 0
 drop_time_node5_drone4_trip0: 10 p[5,4,0] + arr[5,4,0] - dep[5,4,0] = 0
 drop_time_node5_drone4_trip1: 10 p[5,4,1] + arr[5,4,1] - dep[5,4,1] = 0
 drop_time_node6_drone0_trip0: 10 p[6,0,0] + arr[6,0,0] - dep[6,0,0] = 0
 drop_time_node6_drone0_trip1: 10 p[6,0,1] + arr[6,0,1] - dep[6,0,1] = 0
 drop_time_node6_drone1_trip0: 10 p[6,1,0] + arr[6,1,0] - dep[6,1,0] = 0
 drop_time_node6_drone1_trip1: 10 p[6,1,1] + arr[6,1,1] - dep[6,1,1] = 0
 drop_time_node6_drone2_trip0: 10 p[6,2,0] + arr[6,2,0] - dep[6,2,0] = 0
 drop_time_node6_drone2_trip1: 10 p[6,2,1] + arr[6,2,1] - dep[6,2,1] = 0
 drop_time_node6_drone3_trip0: 10 p[6,3,0] + arr[6,3,0] - dep[6,3,0] = 0
 drop_time_node6_drone3_trip1: 10 p[6,3,1] + arr[6,3,1] - dep[6,3,1] = 0
 drop_time_node6_drone4_trip0: 10 p[6,4,0] + arr[6,4,0] - dep[6,4,0] = 0
 drop_time_node6_drone4_trip1: 10 p[6,4,1] + arr[6,4,1] - dep[6,4,1] = 0
 refill_time_drone0_trip1: arr[0,0,0] - dep[0,0,1] + 90 a[0,1] <= 0
 refill_time_drone1_trip1: arr[0,1,0] - dep[0,1,1] + 90 a[1,1] <= 0
 refill_time_drone2_trip1: arr[0,2,0] - dep[0,2,1] + 90 a[2,1] <= 0
 refill_time_drone3_trip1: arr[0,3,0] - dep[0,3,1] + 90 a[3,1] <= 0
 refill_time_drone4_trip1: arr[0,4,0] - dep[0,4,1] + 90 a[4,1] <= 0
 for_drone_0_for_trip_1_to_happen_trip_0_must_happen: a[0,0] - a[0,1] >= 0
 for_drone_1_for_trip_1_to_happen_trip_0_must_happen: a[1,0] - a[1,1] >= 0
 for_drone_2_for_trip_1_to_happen_trip_0_must_happen: a[2,0] - a[2,1] >= 0
 for_drone_3_for_trip_1_to_happen_trip_0_must_happen: a[3,0] - a[3,1] >= 0
 for_drone_4_for_trip_1_to_happen_trip_0_must_happen: a[4,0] - a[4,1] >= 0
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
 drone_4_must_arrive_at_origin_after_it_leaves_it_in_trip_0: - arr[0,4,0]
   + dep[0,4,0] <= 0
 drone_4_must_arrive_at_origin_after_it_leaves_it_in_trip_1: - arr[0,4,1]
   + dep[0,4,1] <= 0
 in_trip_0_drone_0_must_leave_0_if_it_travels_there: x[0,1,0,0]
   + x[0,2,0,0] + x[0,3,0,0] + x[0,4,0,0] + x[0,5,0,0] + x[0,6,0,0]
   - x[1,0,0,0] - x[2,0,0,0] - x[3,0,0,0] - x[4,0,0,0] - x[5,0,0,0]
   - x[6,0,0,0] = 0
 in_trip_0_drone_0_must_leave_1_if_it_travels_there: - x[0,1,0,0]
   + x[1,0,0,0] + x[1,2,0,0] + x[1,3,0,0] + x[1,4,0,0] + x[1,5,0,0]
   + x[1,6,0,0] - x[2,1,0,0] - x[3,1,0,0] - x[4,1,0,0] - x[5,1,0,0]
   - x[6,1,0,0] = 0
 in_trip_0_drone_0_must_leave_2_if_it_travels_there: - x[0,2,0,0]
   - x[1,2,0,0] + x[2,0,0,0] + x[2,1,0,0] + x[2,3,0,0] + x[2,4,0,0]
   + x[2,5,0,0] + x[2,6,0,0] - x[3,2,0,0] - x[4,2,0,0] - x[5,2,0,0]
   - x[6,2,0,0] = 0
 in_trip_0_drone_0_must_leave_3_if_it_travels_there: - x[0,3,0,0]
   - x[1,3,0,0] - x[2,3,0,0] + x[3,0,0,0] + x[3,1,0,0] + x[3,2,0,0]
   + x[3,4,0,0] + x[3,5,0,0] + x[3,6,0,0] - x[4,3,0,0] - x[5,3,0,0]
   - x[6,3,0,0] = 0
 in_trip_0_drone_0_must_leave_4_if_it_travels_there: - x[0,4,0,0]
   - x[1,4,0,0] - x[2,4,0,0] - x[3,4,0,0] + x[4,0,0,0] + x[4,1,0,0]
   + x[4,2,0,0] + x[4,3,0,0] + x[4,5,0,0] + x[4,6,0,0] - x[5,4,0,0]
   - x[6,4,0,0] = 0
 in_trip_0_drone_0_must_leave_5_if_it_travels_there: - x[0,5,0,0]
   - x[1,5,0,0] - x[2,5,0,0] - x[3,5,0,0] - x[4,5,0,0] + x[5,0,0,0]
   + x[5,1,0,0] + x[5,2,0,0] + x[5,3,0,0] + x[5,4,0,0] + x[5,6,0,0]
   - x[6,5,0,0] = 0
 in_trip_0_drone_0_must_leave_6_if_it_travels_there: - x[0,6,0,0]
   - x[1,6,0,0] - x[2,6,0,0] - x[3,6,0,0] - x[4,6,0,0] - x[5,6,0,0]
   + x[6,0,0,0] + x[6,1,0,0] + x[6,2,0,0] + x[6,3,0,0] + x[6,4,0,0]
   + x[6,5,0,0] = 0
 in_trip_1_drone_0_must_leave_0_if_it_travels_there: x[0,1,0,1]
   + x[0,2,0,1] + x[0,3,0,1] + x[0,4,0,1] + x[0,5,0,1] + x[0,6,0,1]
   - x[1,0,0,1] - x[2,0,0,1] - x[3,0,0,1] - x[4,0,0,1] - x[5,0,0,1]
   - x[6,0,0,1] = 0
 in_trip_1_drone_0_must_leave_1_if_it_travels_there: - x[0,1,0,1]
   + x[1,0,0,1] + x[1,2,0,1] + x[1,3,0,1] + x[1,4,0,1] + x[1,5,0,1]
   + x[1,6,0,1] - x[2,1,0,1] - x[3,1,0,1] - x[4,1,0,1] - x[5,1,0,1]
   - x[6,1,0,1] = 0
 in_trip_1_drone_0_must_leave_2_if_it_travels_there: - x[0,2,0,1]
   - x[1,2,0,1] + x[2,0,0,1] + x[2,1,0,1] + x[2,3,0,1] + x[2,4,0,1]
   + x[2,5,0,1] + x[2,6,0,1] - x[3,2,0,1] - x[4,2,0,1] - x[5,2,0,1]
   - x[6,2,0,1] = 0
 in_trip_1_drone_0_must_leave_3_if_it_travels_there: - x[0,3,0,1]
   - x[1,3,0,1] - x[2,3,0,1] + x[3,0,0,1] + x[3,1,0,1] + x[3,2,0,1]
   + x[3,4,0,1] + x[3,5,0,1] + x[3,6,0,1] - x[4,3,0,1] - x[5,3,0,1]
   - x[6,3,0,1] = 0
 in_trip_1_drone_0_must_leave_4_if_it_travels_there: - x[0,4,0,1]
   - x[1,4,0,1] - x[2,4,0,1] - x[3,4,0,1] + x[4,0,0,1] + x[4,1,0,1]
   + x[4,2,0,1] + x[4,3,0,1] + x[4,5,0,1] + x[4,6,0,1] - x[5,4,0,1]
   - x[6,4,0,1] = 0
 in_trip_1_drone_0_must_leave_5_if_it_travels_there: - x[0,5,0,1]
   - x[1,5,0,1] - x[2,5,0,1] - x[3,5,0,1] - x[4,5,0,1] + x[5,0,0,1]
   + x[5,1,0,1] + x[5,2,0,1] + x[5,3,0,1] + x[5,4,0,1] + x[5,6,0,1]
   - x[6,5,0,1] = 0
 in_trip_1_drone_0_must_leave_6_if_it_travels_there: - x[0,6,0,1]
   - x[1,6,0,1] - x[2,6,0,1] - x[3,6,0,1] - x[4,6,0,1] - x[5,6,0,1]
   + x[6,0,0,1] + x[6,1,0,1] + x[6,2,0,1] + x[6,3,0,1] + x[6,4,0,1]
   + x[6,5,0,1] = 0
 in_trip_0_drone_1_must_leave_0_if_it_travels_there: x[0,1,1,0]
   + x[0,2,1,0] + x[0,3,1,0] + x[0,4,1,0] + x[0,5,1,0] + x[0,6,1,0]
   - x[1,0,1,0] - x[2,0,1,0] - x[3,0,1,0] - x[4,0,1,0] - x[5,0,1,0]
   - x[6,0,1,0] = 0
 in_trip_0_drone_1_must_leave_1_if_it_travels_there: - x[0,1,1,0]
   + x[1,0,1,0] + x[1,2,1,0] + x[1,3,1,0] + x[1,4,1,0] + x[1,5,1,0]
   + x[1,6,1,0] - x[2,1,1,0] - x[3,1,1,0] - x[4,1,1,0] - x[5,1,1,0]
   - x[6,1,1,0] = 0
 in_trip_0_drone_1_must_leave_2_if_it_travels_there: - x[0,2,1,0]
   - x[1,2,1,0] + x[2,0,1,0] + x[2,1,1,0] + x[2,3,1,0] + x[2,4,1,0]
   + x[2,5,1,0] + x[2,6,1,0] - x[3,2,1,0] - x[4,2,1,0] - x[5,2,1,0]
   - x[6,2,1,0] = 0
 in_trip_0_drone_1_must_leave_3_if_it_travels_there: - x[0,3,1,0]
   - x[1,3,1,0] - x[2,3,1,0] + x[3,0,1,0] + x[3,1,1,0] + x[3,2,1,0]
   + x[3,4,1,0] + x[3,5,1,0] + x[3,6,1,0] - x[4,3,1,0] - x[5,3,1,0]
   - x[6,3,1,0] = 0
 in_trip_0_drone_1_must_leave_4_if_it_travels_there: - x[0,4,1,0]
   - x[1,4,1,0] - x[2,4,1,0] - x[3,4,1,0] + x[4,0,1,0] + x[4,1,1,0]
   + x[4,2,1,0] + x[4,3,1,0] + x[4,5,1,0] + x[4,6,1,0] - x[5,4,1,0]
   - x[6,4,1,0] = 0
 in_trip_0_drone_1_must_leave_5_if_it_travels_there: - x[0,5,1,0]
   - x[1,5,1,0] - x[2,5,1,0] - x[3,5,1,0] - x[4,5,1,0] + x[5,0,1,0]
   + x[5,1,1,0] + x[5,2,1,0] + x[5,3,1,0] + x[5,4,1,0] + x[5,6,1,0]
   - x[6,5,1,0] = 0
 in_trip_0_drone_1_must_leave_6_if_it_travels_there: - x[0,6,1,0]
   - x[1,6,1,0] - x[2,6,1,0] - x[3,6,1,0] - x[4,6,1,0] - x[5,6,1,0]
   + x[6,0,1,0] + x[6,1,1,0] + x[6,2,1,0] + x[6,3,1,0] + x[6,4,1,0]
   + x[6,5,1,0] = 0
 in_trip_1_drone_1_must_leave_0_if_it_travels_there: x[0,1,1,1]
   + x[0,2,1,1] + x[0,3,1,1] + x[0,4,1,1] + x[0,5,1,1] + x[0,6,1,1]
   - x[1,0,1,1] - x[2,0,1,1] - x[3,0,1,1] - x[4,0,1,1] - x[5,0,1,1]
   - x[6,0,1,1] = 0
 in_trip_1_drone_1_must_leave_1_if_it_travels_there: - x[0,1,1,1]
   + x[1,0,1,1] + x[1,2,1,1] + x[1,3,1,1] + x[1,4,1,1] + x[1,5,1,1]
   + x[1,6,1,1] - x[2,1,1,1] - x[3,1,1,1] - x[4,1,1,1] - x[5,1,1,1]
   - x[6,1,1,1] = 0
 in_trip_1_drone_1_must_leave_2_if_it_travels_there: - x[0,2,1,1]
   - x[1,2,1,1] + x[2,0,1,1] + x[2,1,1,1] + x[2,3,1,1] + x[2,4,1,1]
   + x[2,5,1,1] + x[2,6,1,1] - x[3,2,1,1] - x[4,2,1,1] - x[5,2,1,1]
   - x[6,2,1,1] = 0
 in_trip_1_drone_1_must_leave_3_if_it_travels_there: - x[0,3,1,1]
   - x[1,3,1,1] - x[2,3,1,1] + x[3,0,1,1] + x[3,1,1,1] + x[3,2,1,1]
   + x[3,4,1,1] + x[3,5,1,1] + x[3,6,1,1] - x[4,3,1,1] - x[5,3,1,1]
   - x[6,3,1,1] = 0
 in_trip_1_drone_1_must_leave_4_if_it_travels_there: - x[0,4,1,1]
   - x[1,4,1,1] - x[2,4,1,1] - x[3,4,1,1] + x[4,0,1,1] + x[4,1,1,1]
   + x[4,2,1,1] + x[4,3,1,1] + x[4,5,1,1] + x[4,6,1,1] - x[5,4,1,1]
   - x[6,4,1,1] = 0
 in_trip_1_drone_1_must_leave_5_if_it_travels_there: - x[0,5,1,1]
   - x[1,5,1,1] - x[2,5,1,1] - x[3,5,1,1] - x[4,5,1,1] + x[5,0,1,1]
   + x[5,1,1,1] + x[5,2,1,1] + x[5,3,1,1] + x[5,4,1,1] + x[5,6,1,1]
   - x[6,5,1,1] = 0
 in_trip_1_drone_1_must_leave_6_if_it_travels_there: - x[0,6,1,1]
   - x[1,6,1,1] - x[2,6,1,1] - x[3,6,1,1] - x[4,6,1,1] - x[5,6,1,1]
   + x[6,0,1,1] + x[6,1,1,1] + x[6,2,1,1] + x[6,3,1,1] + x[6,4,1,1]
   + x[6,5,1,1] = 0
 in_trip_0_drone_2_must_leave_0_if_it_travels_there: x[0,1,2,0]
   + x[0,2,2,0] + x[0,3,2,0] + x[0,4,2,0] + x[0,5,2,0] + x[0,6,2,0]
   - x[1,0,2,0] - x[2,0,2,0] - x[3,0,2,0] - x[4,0,2,0] - x[5,0,2,0]
   - x[6,0,2,0] = 0
 in_trip_0_drone_2_must_leave_1_if_it_travels_there: - x[0,1,2,0]
   + x[1,0,2,0] + x[1,2,2,0] + x[1,3,2,0] + x[1,4,2,0] + x[1,5,2,0]
   + x[1,6,2,0] - x[2,1,2,0] - x[3,1,2,0] - x[4,1,2,0] - x[5,1,2,0]
   - x[6,1,2,0] = 0
 in_trip_0_drone_2_must_leave_2_if_it_travels_there: - x[0,2,2,0]
   - x[1,2,2,0] + x[2,0,2,0] + x[2,1,2,0] + x[2,3,2,0] + x[2,4,2,0]
   + x[2,5,2,0] + x[2,6,2,0] - x[3,2,2,0] - x[4,2,2,0] - x[5,2,2,0]
   - x[6,2,2,0] = 0
 in_trip_0_drone_2_must_leave_3_if_it_travels_there: - x[0,3,2,0]
   - x[1,3,2,0] - x[2,3,2,0] + x[3,0,2,0] + x[3,1,2,0] + x[3,2,2,0]
   + x[3,4,2,0] + x[3,5,2,0] + x[3,6,2,0] - x[4,3,2,0] - x[5,3,2,0]
   - x[6,3,2,0] = 0
 in_trip_0_drone_2_must_leave_4_if_it_travels_there: - x[0,4,2,0]
   - x[1,4,2,0] - x[2,4,2,0] - x[3,4,2,0] + x[4,0,2,0] + x[4,1,2,0]
   + x[4,2,2,0] + x[4,3,2,0] + x[4,5,2,0] + x[4,6,2,0] - x[5,4,2,0]
   - x[6,4,2,0] = 0
 in_trip_0_drone_2_must_leave_5_if_it_travels_there: - x[0,5,2,0]
   - x[1,5,2,0] - x[2,5,2,0] - x[3,5,2,0] - x[4,5,2,0] + x[5,0,2,0]
   + x[5,1,2,0] + x[5,2,2,0] + x[5,3,2,0] + x[5,4,2,0] + x[5,6,2,0]
   - x[6,5,2,0] = 0
 in_trip_0_drone_2_must_leave_6_if_it_travels_there: - x[0,6,2,0]
   - x[1,6,2,0] - x[2,6,2,0] - x[3,6,2,0] - x[4,6,2,0] - x[5,6,2,0]
   + x[6,0,2,0] + x[6,1,2,0] + x[6,2,2,0] + x[6,3,2,0] + x[6,4,2,0]
   + x[6,5,2,0] = 0
 in_trip_1_drone_2_must_leave_0_if_it_travels_there: x[0,1,2,1]
   + x[0,2,2,1] + x[0,3,2,1] + x[0,4,2,1] + x[0,5,2,1] + x[0,6,2,1]
   - x[1,0,2,1] - x[2,0,2,1] - x[3,0,2,1] - x[4,0,2,1] - x[5,0,2,1]
   - x[6,0,2,1] = 0
 in_trip_1_drone_2_must_leave_1_if_it_travels_there: - x[0,1,2,1]
   + x[1,0,2,1] + x[1,2,2,1] + x[1,3,2,1] + x[1,4,2,1] + x[1,5,2,1]
   + x[1,6,2,1] - x[2,1,2,1] - x[3,1,2,1] - x[4,1,2,1] - x[5,1,2,1]
   - x[6,1,2,1] = 0
 in_trip_1_drone_2_must_leave_2_if_it_travels_there: - x[0,2,2,1]
   - x[1,2,2,1] + x[2,0,2,1] + x[2,1,2,1] + x[2,3,2,1] + x[2,4,2,1]
   + x[2,5,2,1] + x[2,6,2,1] - x[3,2,2,1] - x[4,2,2,1] - x[5,2,2,1]
   - x[6,2,2,1] = 0
 in_trip_1_drone_2_must_leave_3_if_it_travels_there: - x[0,3,2,1]
   - x[1,3,2,1] - x[2,3,2,1] + x[3,0,2,1] + x[3,1,2,1] + x[3,2,2,1]
   + x[3,4,2,1] + x[3,5,2,1] + x[3,6,2,1] - x[4,3,2,1] - x[5,3,2,1]
   - x[6,3,2,1] = 0
 in_trip_1_drone_2_must_leave_4_if_it_travels_there: - x[0,4,2,1]
   - x[1,4,2,1] - x[2,4,2,1] - x[3,4,2,1] + x[4,0,2,1] + x[4,1,2,1]
   + x[4,2,2,1] + x[4,3,2,1] + x[4,5,2,1] + x[4,6,2,1] - x[5,4,2,1]
   - x[6,4,2,1] = 0
 in_trip_1_drone_2_must_leave_5_if_it_travels_there: - x[0,5,2,1]
   - x[1,5,2,1] - x[2,5,2,1] - x[3,5,2,1] - x[4,5,2,1] + x[5,0,2,1]
   + x[5,1,2,1] + x[5,2,2,1] + x[5,3,2,1] + x[5,4,2,1] + x[5,6,2,1]
   - x[6,5,2,1] = 0
 in_trip_1_drone_2_must_leave_6_if_it_travels_there: - x[0,6,2,1]
   - x[1,6,2,1] - x[2,6,2,1] - x[3,6,2,1] - x[4,6,2,1] - x[5,6,2,1]
   + x[6,0,2,1] + x[6,1,2,1] + x[6,2,2,1] + x[6,3,2,1] + x[6,4,2,1]
   + x[6,5,2,1] = 0
 in_trip_0_drone_3_must_leave_0_if_it_travels_there: x[0,1,3,0]
   + x[0,2,3,0] + x[0,3,3,0] + x[0,4,3,0] + x[0,5,3,0] + x[0,6,3,0]
   - x[1,0,3,0] - x[2,0,3,0] - x[3,0,3,0] - x[4,0,3,0] - x[5,0,3,0]
   - x[6,0,3,0] = 0
 in_trip_0_drone_3_must_leave_1_if_it_travels_there: - x[0,1,3,0]
   + x[1,0,3,0] + x[1,2,3,0] + x[1,3,3,0] + x[1,4,3,0] + x[1,5,3,0]
   + x[1,6,3,0] - x[2,1,3,0] - x[3,1,3,0] - x[4,1,3,0] - x[5,1,3,0]
   - x[6,1,3,0] = 0
 in_trip_0_drone_3_must_leave_2_if_it_travels_there: - x[0,2,3,0]
   - x[1,2,3,0] + x[2,0,3,0] + x[2,1,3,0] + x[2,3,3,0] + x[2,4,3,0]
   + x[2,5,3,0] + x[2,6,3,0] - x[3,2,3,0] - x[4,2,3,0] - x[5,2,3,0]
   - x[6,2,3,0] = 0
 in_trip_0_drone_3_must_leave_3_if_it_travels_there: - x[0,3,3,0]
   - x[1,3,3,0] - x[2,3,3,0] + x[3,0,3,0] + x[3,1,3,0] + x[3,2,3,0]
   + x[3,4,3,0] + x[3,5,3,0] + x[3,6,3,0] - x[4,3,3,0] - x[5,3,3,0]
   - x[6,3,3,0] = 0
 in_trip_0_drone_3_must_leave_4_if_it_travels_there: - x[0,4,3,0]
   - x[1,4,3,0] - x[2,4,3,0] - x[3,4,3,0] + x[4,0,3,0] + x[4,1,3,0]
   + x[4,2,3,0] + x[4,3,3,0] + x[4,5,3,0] + x[4,6,3,0] - x[5,4,3,0]
   - x[6,4,3,0] = 0
 in_trip_0_drone_3_must_leave_5_if_it_travels_there: - x[0,5,3,0]
   - x[1,5,3,0] - x[2,5,3,0] - x[3,5,3,0] - x[4,5,3,0] + x[5,0,3,0]
   + x[5,1,3,0] + x[5,2,3,0] + x[5,3,3,0] + x[5,4,3,0] + x[5,6,3,0]
   - x[6,5,3,0] = 0
 in_trip_0_drone_3_must_leave_6_if_it_travels_there: - x[0,6,3,0]
   - x[1,6,3,0] - x[2,6,3,0] - x[3,6,3,0] - x[4,6,3,0] - x[5,6,3,0]
   + x[6,0,3,0] + x[6,1,3,0] + x[6,2,3,0] + x[6,3,3,0] + x[6,4,3,0]
   + x[6,5,3,0] = 0
 in_trip_1_drone_3_must_leave_0_if_it_travels_there: x[0,1,3,1]
   + x[0,2,3,1] + x[0,3,3,1] + x[0,4,3,1] + x[0,5,3,1] + x[0,6,3,1]
   - x[1,0,3,1] - x[2,0,3,1] - x[3,0,3,1] - x[4,0,3,1] - x[5,0,3,1]
   - x[6,0,3,1] = 0
 in_trip_1_drone_3_must_leave_1_if_it_travels_there: - x[0,1,3,1]
   + x[1,0,3,1] + x[1,2,3,1] + x[1,3,3,1] + x[1,4,3,1] + x[1,5,3,1]
   + x[1,6,3,1] - x[2,1,3,1] - x[3,1,3,1] - x[4,1,3,1] - x[5,1,3,1]
   - x[6,1,3,1] = 0
 in_trip_1_drone_3_must_leave_2_if_it_travels_there: - x[0,2,3,1]
   - x[1,2,3,1] + x[2,0,3,1] + x[2,1,3,1] + x[2,3,3,1] + x[2,4,3,1]
   + x[2,5,3,1] + x[2,6,3,1] - x[3,2,3,1] - x[4,2,3,1] - x[5,2,3,1]
   - x[6,2,3,1] = 0
 in_trip_1_drone_3_must_leave_3_if_it_travels_there: - x[0,3,3,1]
   - x[1,3,3,1] - x[2,3,3,1] + x[3,0,3,1] + x[3,1,3,1] + x[3,2,3,1]
   + x[3,4,3,1] + x[3,5,3,1] + x[3,6,3,1] - x[4,3,3,1] - x[5,3,3,1]
   - x[6,3,3,1] = 0
 in_trip_1_drone_3_must_leave_4_if_it_travels_there: - x[0,4,3,1]
   - x[1,4,3,1] - x[2,4,3,1] - x[3,4,3,1] + x[4,0,3,1] + x[4,1,3,1]
   + x[4,2,3,1] + x[4,3,3,1] + x[4,5,3,1] + x[4,6,3,1] - x[5,4,3,1]
   - x[6,4,3,1] = 0
 in_trip_1_drone_3_must_leave_5_if_it_travels_there: - x[0,5,3,1]
   - x[1,5,3,1] - x[2,5,3,1] - x[3,5,3,1] - x[4,5,3,1] + x[5,0,3,1]
   + x[5,1,3,1] + x[5,2,3,1] + x[5,3,3,1] + x[5,4,3,1] + x[5,6,3,1]
   - x[6,5,3,1] = 0
 in_trip_1_drone_3_must_leave_6_if_it_travels_there: - x[0,6,3,1]
   - x[1,6,3,1] - x[2,6,3,1] - x[3,6,3,1] - x[4,6,3,1] - x[5,6,3,1]
   + x[6,0,3,1] + x[6,1,3,1] + x[6,2,3,1] + x[6,3,3,1] + x[6,4,3,1]
   + x[6,5,3,1] = 0
 in_trip_0_drone_4_must_leave_0_if_it_travels_there: x[0,1,4,0]
   + x[0,2,4,0] + x[0,3,4,0] + x[0,4,4,0] + x[0,5,4,0] + x[0,6,4,0]
   - x[1,0,4,0] - x[2,0,4,0] - x[3,0,4,0] - x[4,0,4,0] - x[5,0,4,0]
   - x[6,0,4,0] = 0
 in_trip_0_drone_4_must_leave_1_if_it_travels_there: - x[0,1,4,0]
   + x[1,0,4,0] + x[1,2,4,0] + x[1,3,4,0] + x[1,4,4,0] + x[1,5,4,0]
   + x[1,6,4,0] - x[2,1,4,0] - x[3,1,4,0] - x[4,1,4,0] - x[5,1,4,0]
   - x[6,1,4,0] = 0
 in_trip_0_drone_4_must_leave_2_if_it_travels_there: - x[0,2,4,0]
   - x[1,2,4,0] + x[2,0,4,0] + x[2,1,4,0] + x[2,3,4,0] + x[2,4,4,0]
   + x[2,5,4,0] + x[2,6,4,0] - x[3,2,4,0] - x[4,2,4,0] - x[5,2,4,0]
   - x[6,2,4,0] = 0
 in_trip_0_drone_4_must_leave_3_if_it_travels_there: - x[0,3,4,0]
   - x[1,3,4,0] - x[2,3,4,0] + x[3,0,4,0] + x[3,1,4,0] + x[3,2,4,0]
   + x[3,4,4,0] + x[3,5,4,0] + x[3,6,4,0] - x[4,3,4,0] - x[5,3,4,0]
   - x[6,3,4,0] = 0
 in_trip_0_drone_4_must_leave_4_if_it_travels_there: - x[0,4,4,0]
   - x[1,4,4,0] - x[2,4,4,0] - x[3,4,4,0] + x[4,0,4,0] + x[4,1,4,0]
   + x[4,2,4,0] + x[4,3,4,0] + x[4,5,4,0] + x[4,6,4,0] - x[5,4,4,0]
   - x[6,4,4,0] = 0
 in_trip_0_drone_4_must_leave_5_if_it_travels_there: - x[0,5,4,0]
   - x[1,5,4,0] - x[2,5,4,0] - x[3,5,4,0] - x[4,5,4,0] + x[5,0,4,0]
   + x[5,1,4,0] + x[5,2,4,0] + x[5,3,4,0] + x[5,4,4,0] + x[5,6,4,0]
   - x[6,5,4,0] = 0
 in_trip_0_drone_4_must_leave_6_if_it_travels_there: - x[0,6,4,0]
   - x[1,6,4,0] - x[2,6,4,0] - x[3,6,4,0] - x[4,6,4,0] - x[5,6,4,0]
   + x[6,0,4,0] + x[6,1,4,0] + x[6,2,4,0] + x[6,3,4,0] + x[6,4,4,0]
   + x[6,5,4,0] = 0
 in_trip_1_drone_4_must_leave_0_if_it_travels_there: x[0,1,4,1]
   + x[0,2,4,1] + x[0,3,4,1] + x[0,4,4,1] + x[0,5,4,1] + x[0,6,4,1]
   - x[1,0,4,1] - x[2,0,4,1] - x[3,0,4,1] - x[4,0,4,1] - x[5,0,4,1]
   - x[6,0,4,1] = 0
 in_trip_1_drone_4_must_leave_1_if_it_travels_there: - x[0,1,4,1]
   + x[1,0,4,1] + x[1,2,4,1] + x[1,3,4,1] + x[1,4,4,1] + x[1,5,4,1]
   + x[1,6,4,1] - x[2,1,4,1] - x[3,1,4,1] - x[4,1,4,1] - x[5,1,4,1]
   - x[6,1,4,1] = 0
 in_trip_1_drone_4_must_leave_2_if_it_travels_there: - x[0,2,4,1]
   - x[1,2,4,1] + x[2,0,4,1] + x[2,1,4,1] + x[2,3,4,1] + x[2,4,4,1]
   + x[2,5,4,1] + x[2,6,4,1] - x[3,2,4,1] - x[4,2,4,1] - x[5,2,4,1]
   - x[6,2,4,1] = 0
 in_trip_1_drone_4_must_leave_3_if_it_travels_there: - x[0,3,4,1]
   - x[1,3,4,1] - x[2,3,4,1] + x[3,0,4,1] + x[3,1,4,1] + x[3,2,4,1]
   + x[3,4,4,1] + x[3,5,4,1] + x[3,6,4,1] - x[4,3,4,1] - x[5,3,4,1]
   - x[6,3,4,1] = 0
 in_trip_1_drone_4_must_leave_4_if_it_travels_there: - x[0,4,4,1]
   - x[1,4,4,1] - x[2,4,4,1] - x[3,4,4,1] + x[4,0,4,1] + x[4,1,4,1]
   + x[4,2,4,1] + x[4,3,4,1] + x[4,5,4,1] + x[4,6,4,1] - x[5,4,4,1]
   - x[6,4,4,1] = 0
 in_trip_1_drone_4_must_leave_5_if_it_travels_there: - x[0,5,4,1]
   - x[1,5,4,1] - x[2,5,4,1] - x[3,5,4,1] - x[4,5,4,1] + x[5,0,4,1]
   + x[5,1,4,1] + x[5,2,4,1] + x[5,3,4,1] + x[5,4,4,1] + x[5,6,4,1]
   - x[6,5,4,1] = 0
 in_trip_1_drone_4_must_leave_6_if_it_travels_there: - x[0,6,4,1]
   - x[1,6,4,1] - x[2,6,4,1] - x[3,6,4,1] - x[4,6,4,1] - x[5,6,4,1]
   + x[6,0,4,1] + x[6,1,4,1] + x[6,2,4,1] + x[6,3,4,1] + x[6,4,4,1]
   + x[6,5,4,1] = 0
 drone_0_must_leave_depot_in_trip0: x[0,1,0,0] + x[0,2,0,0] + x[0,3,0,0]
   + x[0,4,0,0] + x[0,5,0,0] + x[0,6,0,0] - a[0,0] = 0
 drone_0_must_leave_depot_in_trip1: x[0,1,0,1] + x[0,2,0,1] + x[0,3,0,1]
   + x[0,4,0,1] + x[0,5,0,1] + x[0,6,0,1] - a[0,1] = 0
 drone_1_must_leave_depot_in_trip0: x[0,1,1,0] + x[0,2,1,0] + x[0,3,1,0]
   + x[0,4,1,0] + x[0,5,1,0] + x[0,6,1,0] - a[1,0] = 0
 drone_1_must_leave_depot_in_trip1: x[0,1,1,1] + x[0,2,1,1] + x[0,3,1,1]
   + x[0,4,1,1] + x[0,5,1,1] + x[0,6,1,1] - a[1,1] = 0
 drone_2_must_leave_depot_in_trip0: x[0,1,2,0] + x[0,2,2,0] + x[0,3,2,0]
   + x[0,4,2,0] + x[0,5,2,0] + x[0,6,2,0] - a[2,0] = 0
 drone_2_must_leave_depot_in_trip1: x[0,1,2,1] + x[0,2,2,1] + x[0,3,2,1]
   + x[0,4,2,1] + x[0,5,2,1] + x[0,6,2,1] - a[2,1] = 0
 drone_3_must_leave_depot_in_trip0: x[0,1,3,0] + x[0,2,3,0] + x[0,3,3,0]
   + x[0,4,3,0] + x[0,5,3,0] + x[0,6,3,0] - a[3,0] = 0
 drone_3_must_leave_depot_in_trip1: x[0,1,3,1] + x[0,2,3,1] + x[0,3,3,1]
   + x[0,4,3,1] + x[0,5,3,1] + x[0,6,3,1] - a[3,1] = 0
 drone_4_must_leave_depot_in_trip0: x[0,1,4,0] + x[0,2,4,0] + x[0,3,4,0]
   + x[0,4,4,0] + x[0,5,4,0] + x[0,6,4,0] - a[4,0] = 0
 drone_4_must_leave_depot_in_trip1: x[0,1,4,1] + x[0,2,4,1] + x[0,3,4,1]
   + x[0,4,4,1] + x[0,5,4,1] + x[0,6,4,1] - a[4,1] = 0
 drone_0_must_return_to_depot_in_trip0: x[1,0,0,0] + x[2,0,0,0]
   + x[3,0,0,0] + x[4,0,0,0] + x[5,0,0,0] + x[6,0,0,0] - a[0,0] = 0
 drone_0_must_return_to_depot_in_trip1: x[1,0,0,1] + x[2,0,0,1]
   + x[3,0,0,1] + x[4,0,0,1] + x[5,0,0,1] + x[6,0,0,1] - a[0,1] = 0
 drone_1_must_return_to_depot_in_trip0: x[1,0,1,0] + x[2,0,1,0]
   + x[3,0,1,0] + x[4,0,1,0] + x[5,0,1,0] + x[6,0,1,0] - a[1,0] = 0
 drone_1_must_return_to_depot_in_trip1: x[1,0,1,1] + x[2,0,1,1]
   + x[3,0,1,1] + x[4,0,1,1] + x[5,0,1,1] + x[6,0,1,1] - a[1,1] = 0
 drone_2_must_return_to_depot_in_trip0: x[1,0,2,0] + x[2,0,2,0]
   + x[3,0,2,0] + x[4,0,2,0] + x[5,0,2,0] + x[6,0,2,0] - a[2,0] = 0
 drone_2_must_return_to_depot_in_trip1: x[1,0,2,1] + x[2,0,2,1]
   + x[3,0,2,1] + x[4,0,2,1] + x[5,0,2,1] + x[6,0,2,1] - a[2,1] = 0
 drone_3_must_return_to_depot_in_trip0: x[1,0,3,0] + x[2,0,3,0]
   + x[3,0,3,0] + x[4,0,3,0] + x[5,0,3,0] + x[6,0,3,0] - a[3,0] = 0
 drone_3_must_return_to_depot_in_trip1: x[1,0,3,1] + x[2,0,3,1]
   + x[3,0,3,1] + x[4,0,3,1] + x[5,0,3,1] + x[6,0,3,1] - a[3,1] = 0
 drone_4_must_return_to_depot_in_trip0: x[1,0,4,0] + x[2,0,4,0]
   + x[3,0,4,0] + x[4,0,4,0] + x[5,0,4,0] + x[6,0,4,0] - a[4,0] = 0
 drone_4_must_return_to_depot_in_trip1: x[1,0,4,1] + x[2,0,4,1]
   + x[3,0,4,1] + x[4,0,4,1] + x[5,0,4,1] + x[6,0,4,1] - a[4,1] = 0
 Total_time_greater_than_final_time_of_drone0: T - arr[0,0,1] >= 0
 Total_time_greater_than_final_time_of_drone1: T - arr[0,1,1] >= 0
 Total_time_greater_than_final_time_of_drone2: T - arr[0,2,1] >= 0
 Total_time_greater_than_final_time_of_drone3: T - arr[0,3,1] >= 0
 Total_time_greater_than_final_time_of_drone4: T - arr[0,4,1] >= 0
 Completion_time_node1: - dep[1,0,0] + Tcomp[1] >= 0
 Completion_time_node1: - dep[1,1,0] + Tcomp[1] >= 0
 Completion_time_node1: - dep[1,2,0] + Tcomp[1] >= 0
 Completion_time_node1: - dep[1,3,0] + Tcomp[1] >= 0
 Completion_time_node1: - dep[1,4,0] + Tcomp[1] >= 0
 Completion_time_node1: - dep[1,0,1] + Tcomp[1] >= 0
 Completion_time_node1: - dep[1,1,1] + Tcomp[1] >= 0
 Completion_time_node1: - dep[1,2,1] + Tcomp[1] >= 0
 Completion_time_node1: - dep[1,3,1] + Tcomp[1] >= 0
 Completion_time_node1: - dep[1,4,1] + Tcomp[1] >= 0
 Completion_time_node2: - dep[2,0,0] + Tcomp[2] >= 0
 Completion_time_node2: - dep[2,1,0] + Tcomp[2] >= 0
 Completion_time_node2: - dep[2,2,0] + Tcomp[2] >= 0
 Completion_time_node2: - dep[2,3,0] + Tcomp[2] >= 0
 Completion_time_node2: - dep[2,4,0] + Tcomp[2] >= 0
 Completion_time_node2: - dep[2,0,1] + Tcomp[2] >= 0
 Completion_time_node2: - dep[2,1,1] + Tcomp[2] >= 0
 Completion_time_node2: - dep[2,2,1] + Tcomp[2] >= 0
 Completion_time_node2: - dep[2,3,1] + Tcomp[2] >= 0
 Completion_time_node2: - dep[2,4,1] + Tcomp[2] >= 0
 Completion_time_node3: - dep[3,0,0] + Tcomp[3] >= 0
 Completion_time_node3: - dep[3,1,0] + Tcomp[3] >= 0
 Completion_time_node3: - dep[3,2,0] + Tcomp[3] >= 0
 Completion_time_node3: - dep[3,3,0] + Tcomp[3] >= 0
 Completion_time_node3: - dep[3,4,0] + Tcomp[3] >= 0
 Completion_time_node3: - dep[3,0,1] + Tcomp[3] >= 0
 Completion_time_node3: - dep[3,1,1] + Tcomp[3] >= 0
 Completion_time_node3: - dep[3,2,1] + Tcomp[3] >= 0
 Completion_time_node3: - dep[3,3,1] + Tcomp[3] >= 0
 Completion_time_node3: - dep[3,4,1] + Tcomp[3] >= 0
 Completion_time_node4: - dep[4,0,0] + Tcomp[4] >= 0
 Completion_time_node4: - dep[4,1,0] + Tcomp[4] >= 0
 Completion_time_node4: - dep[4,2,0] + Tcomp[4] >= 0
 Completion_time_node4: - dep[4,3,0] + Tcomp[4] >= 0
 Completion_time_node4: - dep[4,4,0] + Tcomp[4] >= 0
 Completion_time_node4: - dep[4,0,1] + Tcomp[4] >= 0
 Completion_time_node4: - dep[4,1,1] + Tcomp[4] >= 0
 Completion_time_node4: - dep[4,2,1] + Tcomp[4] >= 0
 Completion_time_node4: - dep[4,3,1] + Tcomp[4] >= 0
 Completion_time_node4: - dep[4,4,1] + Tcomp[4] >= 0
 Completion_time_node5: - dep[5,0,0] + Tcomp[5] >= 0
 Completion_time_node5: - dep[5,1,0] + Tcomp[5] >= 0
 Completion_time_node5: - dep[5,2,0] + Tcomp[5] >= 0
 Completion_time_node5: - dep[5,3,0] + Tcomp[5] >= 0
 Completion_time_node5: - dep[5,4,0] + Tcomp[5] >= 0
 Completion_time_node5: - dep[5,0,1] + Tcomp[5] >= 0
 Completion_time_node5: - dep[5,1,1] + Tcomp[5] >= 0
 Completion_time_node5: - dep[5,2,1] + Tcomp[5] >= 0
 Completion_time_node5: - dep[5,3,1] + Tcomp[5] >= 0
 Completion_time_node5: - dep[5,4,1] + Tcomp[5] >= 0
 Completion_time_node6: - dep[6,0,0] + Tcomp[6] >= 0
 Completion_time_node6: - dep[6,1,0] + Tcomp[6] >= 0
 Completion_time_node6: - dep[6,2,0] + Tcomp[6] >= 0
 Completion_time_node6: - dep[6,3,0] + Tcomp[6] >= 0
 Completion_time_node6: - dep[6,4,0] + Tcomp[6] >= 0
 Completion_time_node6: - dep[6,0,1] + Tcomp[6] >= 0
 Completion_time_node6: - dep[6,1,1] + Tcomp[6] >= 0
 Completion_time_node6: - dep[6,2,1] + Tcomp[6] >= 0
 Completion_time_node6: - dep[6,3,1] + Tcomp[6] >= 0
 Completion_time_node6: - dep[6,4,1] + Tcomp[6] >= 0
Bounds
 p[0,0,0] <= 7
 p[0,0,1] <= 7
 p[0,1,0] <= 7
 p[0,1,1] <= 7
 p[0,2,0] <= 7
 p[0,2,1] <= 7
 p[0,3,0] <= 7
 p[0,3,1] <= 7
 p[0,4,0] <= 7
 p[0,4,1] <= 7
 p[1,0,0] <= 7
 p[1,0,1] <= 7
 p[1,1,0] <= 7
 p[1,1,1] <= 7
 p[1,2,0] <= 7
 p[1,2,1] <= 7
 p[1,3,0] <= 7
 p[1,3,1] <= 7
 p[1,4,0] <= 7
 p[1,4,1] <= 7
 p[2,0,0] <= 7
 p[2,0,1] <= 7
 p[2,1,0] <= 7
 p[2,1,1] <= 7
 p[2,2,0] <= 7
 p[2,2,1] <= 7
 p[2,3,0] <= 7
 p[2,3,1] <= 7
 p[2,4,0] <= 7
 p[2,4,1] <= 7
 p[3,0,0] <= 7
 p[3,0,1] <= 7
 p[3,1,0] <= 7
 p[3,1,1] <= 7
 p[3,2,0] <= 7
 p[3,2,1] <= 7
 p[3,3,0] <= 7
 p[3,3,1] <= 7
 p[3,4,0] <= 7
 p[3,4,1] <= 7
 p[4,0,0] <= 7
 p[4,0,1] <= 7
 p[4,1,0] <= 7
 p[4,1,1] <= 7
 p[4,2,0] <= 7
 p[4,2,1] <= 7
 p[4,3,0] <= 7
 p[4,3,1] <= 7
 p[4,4,0] <= 7
 p[4,4,1] <= 7
 p[5,0,0] <= 7
 p[5,0,1] <= 7
 p[5,1,0] <= 7
 p[5,1,1] <= 7
 p[5,2,0] <= 7
 p[5,2,1] <= 7
 p[5,3,0] <= 7
 p[5,3,1] <= 7
 p[5,4,0] <= 7
 p[5,4,1] <= 7
 p[6,0,0] <= 7
 p[6,0,1] <= 7
 p[6,1,0] <= 7
 p[6,1,1] <= 7
 p[6,2,0] <= 7
 p[6,2,1] <= 7
 p[6,3,0] <= 7
 p[6,3,1] <= 7
 p[6,4,0] <= 7
 p[6,4,1] <= 7
Binaries
 x[0,0,0,0] x[0,0,0,1] x[0,0,1,0] x[0,0,1,1] x[0,0,2,0] x[0,0,2,1]
 x[0,0,3,0] x[0,0,3,1] x[0,0,4,0] x[0,0,4,1] x[0,1,0,0] x[0,1,0,1]
 x[0,1,1,0] x[0,1,1,1] x[0,1,2,0] x[0,1,2,1] x[0,1,3,0] x[0,1,3,1]
 x[0,1,4,0] x[0,1,4,1] x[0,2,0,0] x[0,2,0,1] x[0,2,1,0] x[0,2,1,1]
 x[0,2,2,0] x[0,2,2,1] x[0,2,3,0] x[0,2,3,1] x[0,2,4,0] x[0,2,4,1]
 x[0,3,0,0] x[0,3,0,1] x[0,3,1,0] x[0,3,1,1] x[0,3,2,0] x[0,3,2,1]
 x[0,3,3,0] x[0,3,3,1] x[0,3,4,0] x[0,3,4,1] x[0,4,0,0] x[0,4,0,1]
 x[0,4,1,0] x[0,4,1,1] x[0,4,2,0] x[0,4,2,1] x[0,4,3,0] x[0,4,3,1]
 x[0,4,4,0] x[0,4,4,1] x[0,5,0,0] x[0,5,0,1] x[0,5,1,0] x[0,5,1,1]
 x[0,5,2,0] x[0,5,2,1] x[0,5,3,0] x[0,5,3,1] x[0,5,4,0] x[0,5,4,1]
 x[0,6,0,0] x[0,6,0,1] x[0,6,1,0] x[0,6,1,1] x[0,6,2,0] x[0,6,2,1]
 x[0,6,3,0] x[0,6,3,1] x[0,6,4,0] x[0,6,4,1] x[1,0,0,0] x[1,0,0,1]
 x[1,0,1,0] x[1,0,1,1] x[1,0,2,0] x[1,0,2,1] x[1,0,3,0] x[1,0,3,1]
 x[1,0,4,0] x[1,0,4,1] x[1,1,0,0] x[1,1,0,1] x[1,1,1,0] x[1,1,1,1]
 x[1,1,2,0] x[1,1,2,1] x[1,1,3,0] x[1,1,3,1] x[1,1,4,0] x[1,1,4,1]
 x[1,2,0,0] x[1,2,0,1] x[1,2,1,0] x[1,2,1,1] x[1,2,2,0] x[1,2,2,1]
 x[1,2,3,0] x[1,2,3,1] x[1,2,4,0] x[1,2,4,1] x[1,3,0,0] x[1,3,0,1]
 x[1,3,1,0] x[1,3,1,1] x[1,3,2,0] x[1,3,2,1] x[1,3,3,0] x[1,3,3,1]
 x[1,3,4,0] x[1,3,4,1] x[1,4,0,0] x[1,4,0,1] x[1,4,1,0] x[1,4,1,1]
 x[1,4,2,0] x[1,4,2,1] x[1,4,3,0] x[1,4,3,1] x[1,4,4,0] x[1,4,4,1]
 x[1,5,0,0] x[1,5,0,1] x[1,5,1,0] x[1,5,1,1] x[1,5,2,0] x[1,5,2,1]
 x[1,5,3,0] x[1,5,3,1] x[1,5,4,0] x[1,5,4,1] x[1,6,0,0] x[1,6,0,1]
 x[1,6,1,0] x[1,6,1,1] x[1,6,2,0] x[1,6,2,1] x[1,6,3,0] x[1,6,3,1]
 x[1,6,4,0] x[1,6,4,1] x[2,0,0,0] x[2,0,0,1] x[2,0,1,0] x[2,0,1,1]
 x[2,0,2,0] x[2,0,2,1] x[2,0,3,0] x[2,0,3,1] x[2,0,4,0] x[2,0,4,1]
 x[2,1,0,0] x[2,1,0,1] x[2,1,1,0] x[2,1,1,1] x[2,1,2,0] x[2,1,2,1]
 x[2,1,3,0] x[2,1,3,1] x[2,1,4,0] x[2,1,4,1] x[2,2,0,0] x[2,2,0,1]
 x[2,2,1,0] x[2,2,1,1] x[2,2,2,0] x[2,2,2,1] x[2,2,3,0] x[2,2,3,1]
 x[2,2,4,0] x[2,2,4,1] x[2,3,0,0] x[2,3,0,1] x[2,3,1,0] x[2,3,1,1]
 x[2,3,2,0] x[2,3,2,1] x[2,3,3,0] x[2,3,3,1] x[2,3,4,0] x[2,3,4,1]
 x[2,4,0,0] x[2,4,0,1] x[2,4,1,0] x[2,4,1,1] x[2,4,2,0] x[2,4,2,1]
 x[2,4,3,0] x[2,4,3,1] x[2,4,4,0] x[2,4,4,1] x[2,5,0,0] x[2,5,0,1]
 x[2,5,1,0] x[2,5,1,1] x[2,5,2,0] x[2,5,2,1] x[2,5,3,0] x[2,5,3,1]
 x[2,5,4,0] x[2,5,4,1] x[2,6,0,0] x[2,6,0,1] x[2,6,1,0] x[2,6,1,1]
 x[2,6,2,0] x[2,6,2,1] x[2,6,3,0] x[2,6,3,1] x[2,6,4,0] x[2,6,4,1]
 x[3,0,0,0] x[3,0,0,1] x[3,0,1,0] x[3,0,1,1] x[3,0,2,0] x[3,0,2,1]
 x[3,0,3,0] x[3,0,3,1] x[3,0,4,0] x[3,0,4,1] x[3,1,0,0] x[3,1,0,1]
 x[3,1,1,0] x[3,1,1,1] x[3,1,2,0] x[3,1,2,1] x[3,1,3,0] x[3,1,3,1]
 x[3,1,4,0] x[3,1,4,1] x[3,2,0,0] x[3,2,0,1] x[3,2,1,0] x[3,2,1,1]
 x[3,2,2,0] x[3,2,2,1] x[3,2,3,0] x[3,2,3,1] x[3,2,4,0] x[3,2,4,1]
 x[3,3,0,0] x[3,3,0,1] x[3,3,1,0] x[3,3,1,1] x[3,3,2,0] x[3,3,2,1]
 x[3,3,3,0] x[3,3,3,1] x[3,3,4,0] x[3,3,4,1] x[3,4,0,0] x[3,4,0,1]
 x[3,4,1,0] x[3,4,1,1] x[3,4,2,0] x[3,4,2,1] x[3,4,3,0] x[3,4,3,1]
 x[3,4,4,0] x[3,4,4,1] x[3,5,0,0] x[3,5,0,1] x[3,5,1,0] x[3,5,1,1]
 x[3,5,2,0] x[3,5,2,1] x[3,5,3,0] x[3,5,3,1] x[3,5,4,0] x[3,5,4,1]
 x[3,6,0,0] x[3,6,0,1] x[3,6,1,0] x[3,6,1,1] x[3,6,2,0] x[3,6,2,1]
 x[3,6,3,0] x[3,6,3,1] x[3,6,4,0] x[3,6,4,1] x[4,0,0,0] x[4,0,0,1]
 x[4,0,1,0] x[4,0,1,1] x[4,0,2,0] x[4,0,2,1] x[4,0,3,0] x[4,0,3,1]
 x[4,0,4,0] x[4,0,4,1] x[4,1,0,0] x[4,1,0,1] x[4,1,1,0] x[4,1,1,1]
 x[4,1,2,0] x[4,1,2,1] x[4,1,3,0] x[4,1,3,1] x[4,1,4,0] x[4,1,4,1]
 x[4,2,0,0] x[4,2,0,1] x[4,2,1,0] x[4,2,1,1] x[4,2,2,0] x[4,2,2,1]
 x[4,2,3,0] x[4,2,3,1] x[4,2,4,0] x[4,2,4,1] x[4,3,0,0] x[4,3,0,1]
 x[4,3,1,0] x[4,3,1,1] x[4,3,2,0] x[4,3,2,1] x[4,3,3,0] x[4,3,3,1]
 x[4,3,4,0] x[4,3,4,1] x[4,4,0,0] x[4,4,0,1] x[4,4,1,0] x[4,4,1,1]
 x[4,4,2,0] x[4,4,2,1] x[4,4,3,0] x[4,4,3,1] x[4,4,4,0] x[4,4,4,1]
 x[4,5,0,0] x[4,5,0,1] x[4,5,1,0] x[4,5,1,1] x[4,5,2,0] x[4,5,2,1]
 x[4,5,3,0] x[4,5,3,1] x[4,5,4,0] x[4,5,4,1] x[4,6,0,0] x[4,6,0,1]
 x[4,6,1,0] x[4,6,1,1] x[4,6,2,0] x[4,6,2,1] x[4,6,3,0] x[4,6,3,1]
 x[4,6,4,0] x[4,6,4,1] x[5,0,0,0] x[5,0,0,1] x[5,0,1,0] x[5,0,1,1]
 x[5,0,2,0] x[5,0,2,1] x[5,0,3,0] x[5,0,3,1] x[5,0,4,0] x[5,0,4,1]
 x[5,1,0,0] x[5,1,0,1] x[5,1,1,0] x[5,1,1,1] x[5,1,2,0] x[5,1,2,1]
 x[5,1,3,0] x[5,1,3,1] x[5,1,4,0] x[5,1,4,1] x[5,2,0,0] x[5,2,0,1]
 x[5,2,1,0] x[5,2,1,1] x[5,2,2,0] x[5,2,2,1] x[5,2,3,0] x[5,2,3,1]
 x[5,2,4,0] x[5,2,4,1] x[5,3,0,0] x[5,3,0,1] x[5,3,1,0] x[5,3,1,1]
 x[5,3,2,0] x[5,3,2,1] x[5,3,3,0] x[5,3,3,1] x[5,3,4,0] x[5,3,4,1]
 x[5,4,0,0] x[5,4,0,1] x[5,4,1,0] x[5,4,1,1] x[5,4,2,0] x[5,4,2,1]
 x[5,4,3,0] x[5,4,3,1] x[5,4,4,0] x[5,4,4,1] x[5,5,0,0] x[5,5,0,1]
 x[5,5,1,0] x[5,5,1,1] x[5,5,2,0] x[5,5,2,1] x[5,5,3,0] x[5,5,3,1]
 x[5,5,4,0] x[5,5,4,1] x[5,6,0,0] x[5,6,0,1] x[5,6,1,0] x[5,6,1,1]
 x[5,6,2,0] x[5,6,2,1] x[5,6,3,0] x[5,6,3,1] x[5,6,4,0] x[5,6,4,1]
 x[6,0,0,0] x[6,0,0,1] x[6,0,1,0] x[6,0,1,1] x[6,0,2,0] x[6,0,2,1]
 x[6,0,3,0] x[6,0,3,1] x[6,0,4,0] x[6,0,4,1] x[6,1,0,0] x[6,1,0,1]
 x[6,1,1,0] x[6,1,1,1] x[6,1,2,0] x[6,1,2,1] x[6,1,3,0] x[6,1,3,1]
 x[6,1,4,0] x[6,1,4,1] x[6,2,0,0] x[6,2,0,1] x[6,2,1,0] x[6,2,1,1]
 x[6,2,2,0] x[6,2,2,1] x[6,2,3,0] x[6,2,3,1] x[6,2,4,0] x[6,2,4,1]
 x[6,3,0,0] x[6,3,0,1] x[6,3,1,0] x[6,3,1,1] x[6,3,2,0] x[6,3,2,1]
 x[6,3,3,0] x[6,3,3,1] x[6,3,4,0] x[6,3,4,1] x[6,4,0,0] x[6,4,0,1]
 x[6,4,1,0] x[6,4,1,1] x[6,4,2,0] x[6,4,2,1] x[6,4,3,0] x[6,4,3,1]
 x[6,4,4,0] x[6,4,4,1] x[6,5,0,0] x[6,5,0,1] x[6,5,1,0] x[6,5,1,1]
 x[6,5,2,0] x[6,5,2,1] x[6,5,3,0] x[6,5,3,1] x[6,5,4,0] x[6,5,4,1]
 x[6,6,0,0] x[6,6,0,1] x[6,6,1,0] x[6,6,1,1] x[6,6,2,0] x[6,6,2,1]
 x[6,6,3,0] x[6,6,3,1] x[6,6,4,0] x[6,6,4,1] a[0,0] a[0,1] a[1,0] a[1,1]
 a[2,0] a[2,1] a[3,0] a[3,1] a[4,0] a[4,1]
End
