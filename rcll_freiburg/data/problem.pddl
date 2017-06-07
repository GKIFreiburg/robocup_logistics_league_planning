(define (problem rcll-production-steps_task)
(:domain rcll-production-steps)
(:objects
    p70 - product
    r1 r2 r3 - robot
    silver_base_p70 blue_ring_p70 orange_ring_p70 yellow_ring_p70 black_cap_p70 gate2_delivery_p70 - step
)
(:init
    (cap-buffered cs2)
    (cap-buffered cs1)
    (has-step p70 silver_base_p70)
    (has-step p70 blue_ring_p70)
    (has-step p70 orange_ring_p70)
    (has-step p70 yellow_ring_p70)
    (has-step p70 black_cap_p70)
    (has-step p70 gate2_delivery_p70)
    (initial-step silver_base_p70)
    (input-location cs1_in cs1)
    (input-location cs2_in cs2)
    (input-location rs1_in rs1)
    (input-location rs2_in rs2)
    (input-location ds_in ds)
    (location-occupied )
    (location-occupied rs2_out)
    (location-occupied rs1_out)
    (location-occupied bs_out)
    (material-at bs_out)
    (output-full bs_out)
    (output-full rs2_out)
    (output-location cs1_out cs1)
    (output-location cs2_out cs2)
    (output-location rs1_out rs1)
    (output-location rs2_out rs2)
    (output-location bs_out bs)
    (product-at p70 rs2_out)
    (robot-at r1 )
    (robot-at r2 )
    (robot-at r3 )
    (robot-at r1 rs2_out)
    (robot-at r3 rs1_out)
    (robot-at r2 bs_out)
    (robot-precedes r1 r2)
    (robot-precedes r1 r3)
    (robot-precedes r2 r3)
    (robot-recently-moved r1)
    (robot-recently-moved r3)
    (robot-recently-moved r2)
    (step-at-machine silver_base_p70 bs)
    (step-at-machine blue_ring_p70 rs2)
    (step-at-machine orange_ring_p70 rs2)
    (step-at-machine yellow_ring_p70 rs1)
    (step-at-machine black_cap_p70 cs2)
    (step-at-machine gate2_delivery_p70 ds)
    (step-completed silver_base_p70)
    (step-completed orange_ring_p70)
    (step-precedes silver_base_p70 blue_ring_p70)
    (step-precedes blue_ring_p70 orange_ring_p70)
    (step-precedes orange_ring_p70 yellow_ring_p70)
    (step-precedes yellow_ring_p70 black_cap_p70)
    (step-precedes black_cap_p70 gate2_delivery_p70)
    (= (material-required blue_ring_p70) 1)
    (= (material-required orange_ring_p70) 0)
    (= (material-required yellow_ring_p70) 2)
    (= (material-stored rs2) 1)
    (= (material-stored rs1) 2)
    (= (path-length bs_out cs1_in) 25.1942)
    (= (path-length bs_out cs1_out) 37.3719)
    (= (path-length bs_out cs2_in) 46.4659)
    (= (path-length bs_out cs2_out) 62.1429)
    (= (path-length bs_out ds_in) 36.0183)
    (= (path-length bs_out rs1_in) 32.1011)
    (= (path-length bs_out rs1_out) 18.8066)
    (= (path-length bs_out rs2_in) 39.8359)
    (= (path-length bs_out rs2_out) 47.1791)
    (= (path-length cs1_in bs_out) 25.1942)
    (= (path-length cs1_in cs1_out) 15.6148)
    (= (path-length cs1_in cs2_in) 31.6458)
    (= (path-length cs1_in cs2_out) 47.8101)
    (= (path-length cs1_in ds_in) 14.2612)
    (= (path-length cs1_in rs1_in) 21.7388)
    (= (path-length cs1_in rs1_out) 17.3601)
    (= (path-length cs1_in rs2_in) 37.197)
    (= (path-length cs1_in rs2_out) 32.8463)
    (= (path-length cs1_out bs_out) 37.3719)
    (= (path-length cs1_out cs1_in) 15.6148)
    (= (path-length cs1_out cs2_in) 17.436)
    (= (path-length cs1_out cs2_out) 35.2746)
    (= (path-length cs1_out ds_in) 1.63504)
    (= (path-length cs1_out rs1_in) 17.7004)
    (= (path-length cs1_out rs1_out) 27.1899)
    (= (path-length cs1_out rs2_in) 32.6512)
    (= (path-length cs1_out rs2_out) 28.3005)
    (= (path-length cs2_in bs_out) 46.4659)
    (= (path-length cs2_in cs1_in) 31.6458)
    (= (path-length cs2_in cs1_out) 17.436)
    (= (path-length cs2_in cs2_out) 27.4913)
    (= (path-length cs2_in ds_in) 17.666)
    (= (path-length cs2_in rs1_in) 22.4566)
    (= (path-length cs2_in rs1_out) 31.9462)
    (= (path-length cs2_in rs2_in) 20.0761)
    (= (path-length cs2_in rs2_out) 13.2902)
    (= (path-length cs2_out bs_out) 62.1429)
    (= (path-length cs2_out cs1_in) 47.8101)
    (= (path-length cs2_out cs1_out) 35.2746)
    (= (path-length cs2_out cs2_in) 27.4913)
    (= (path-length cs2_out ds_in) 35.5047)
    (= (path-length cs2_out rs1_in) 38.1337)
    (= (path-length cs2_out rs1_out) 47.6232)
    (= (path-length cs2_out rs2_in) 28.3865)
    (= (path-length cs2_out rs2_out) 20.6194)
    (= (path-length ds_in bs_out) 36.0183)
    (= (path-length ds_in cs1_in) 14.2612)
    (= (path-length ds_in cs1_out) 1.63504)
    (= (path-length ds_in cs2_in) 17.666)
    (= (path-length ds_in cs2_out) 35.5047)
    (= (path-length ds_in rs1_in) 17.9304)
    (= (path-length ds_in rs1_out) 27.42)
    (= (path-length ds_in rs2_in) 32.8812)
    (= (path-length ds_in rs2_out) 28.5305)
    (= (path-length rs1_in bs_out) 32.1011)
    (= (path-length rs1_in cs1_in) 21.7388)
    (= (path-length rs1_in cs1_out) 17.7004)
    (= (path-length rs1_in cs2_in) 22.4566)
    (= (path-length rs1_in cs2_out) 38.1337)
    (= (path-length rs1_in ds_in) 17.9304)
    (= (path-length rs1_in rs1_out) 14.9157)
    (= (path-length rs1_in rs2_in) 21.1873)
    (= (path-length rs1_in rs2_out) 23.1698)
    (= (path-length rs1_out bs_out) 18.8066)
    (= (path-length rs1_out cs1_in) 17.3601)
    (= (path-length rs1_out cs1_out) 27.19)
    (= (path-length rs1_out cs2_in) 31.9462)
    (= (path-length rs1_out cs2_out) 47.6232)
    (= (path-length rs1_out ds_in) 27.42)
    (= (path-length rs1_out rs1_in) 14.9157)
    (= (path-length rs1_out rs2_in) 22.6506)
    (= (path-length rs1_out rs2_out) 32.6594)
    (= (path-length rs2_in bs_out) 39.8359)
    (= (path-length rs2_in cs1_in) 37.197)
    (= (path-length rs2_in cs1_out) 32.6512)
    (= (path-length rs2_in cs2_in) 20.0761)
    (= (path-length rs2_in cs2_out) 28.3865)
    (= (path-length rs2_in ds_in) 32.8812)
    (= (path-length rs2_in rs1_in) 21.1873)
    (= (path-length rs2_in rs1_out) 22.6506)
    (= (path-length rs2_in rs2_out) 20.0974)
    (= (path-length rs2_out bs_out) 47.1791)
    (= (path-length rs2_out cs1_in) 32.8463)
    (= (path-length rs2_out cs1_out) 28.3005)
    (= (path-length rs2_out cs2_in) 13.2902)
    (= (path-length rs2_out cs2_out) 20.6194)
    (= (path-length rs2_out ds_in) 28.5305)
    (= (path-length rs2_out rs1_in) 23.1698)
    (= (path-length rs2_out rs1_out) 32.6594)
    (= (path-length rs2_out rs2_in) 20.0974)
    (= (path-length start bs_out) 8.15059)
    (= (path-length start cs1_in) 21.1005)
    (= (path-length start cs1_out) 33.2782)
    (= (path-length start cs2_in) 41.2626)
    (= (path-length start cs2_out) 56.9396)
    (= (path-length start ds_in) 31.9246)
    (= (path-length start rs1_in) 24.2321)
    (= (path-length start rs1_out) 10.9376)
    (= (path-length start rs2_in) 31.9669)
    (= (path-length start rs2_out) 41.9758)
)
(:goal (and
    (step-completed silver_base_p70)
    (step-completed blue_ring_p70)
    (step-completed orange_ring_p70)
    (step-completed yellow_ring_p70)
    (step-completed black_cap_p70)
    (step-completed gate2_delivery_p70)
)))
