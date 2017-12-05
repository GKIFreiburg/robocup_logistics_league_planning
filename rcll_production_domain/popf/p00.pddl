(define (problem rcll-production-steps_task)
(:domain rcll-production-steps)
(:objects
    r1 r2 r3 - robot
    p10 - product
    silver_base_p10 grey_cap_p10 gate2_delivery_p10 - step
    p70 - product
    silver_base_p70 blue_ring_p70 orange_ring_p70 yellow_ring_p70 black_cap_p70 gate2_delivery_p70 - step
)
(:init
    (add-one zero one)
    (add-one one two)
    (add-one two three)
    (subtract three zero three)
    (subtract two zero two)
    (subtract one zero one)
    (subtract zero zero zero)
    (subtract three one two)
    (subtract two one one)
    (subtract one one zero)
    (subtract three two one)
    (subtract two two zero)
    (less-or-equal zero three)
    (less-or-equal one three)
    (less-or-equal two three)
    (less-or-equal three three)
    (less-or-equal zero two)
    (less-or-equal one two)
    (less-or-equal two two)
    (less-or-equal zero one)
    (less-or-equal one one)
    (less-or-equal zero zero)
    
    (has-step p10 silver_base_p10)
    (has-step p10 grey_cap_p10)
    (has-step p10 gate2_delivery_p10)
    (initial-step silver_base_p10)
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
    (output-location cs1_out cs1)
    (output-location cs2_out cs2)
    (output-location rs1_out rs1)
    (output-location rs2_out rs2)
    (output-location bs_out bs)
    (output-location bs_in bs)
    (first-robot r1)
    (robot-precedes r1 r2)
    (robot-precedes r2 r3)
    (step-at-station silver_base_p10 bs)
    (step-at-station grey_cap_p10 cs1)
    (step-at-station gate2_delivery_p10 ds)
    (step-precedes silver_base_p10 grey_cap_p10)
    (step-precedes grey_cap_p10 gate2_delivery_p10)
    (step-at-station silver_base_p70 bs)
    (step-at-station blue_ring_p70 rs2)
    (step-at-station orange_ring_p70 rs2)
    (step-at-station yellow_ring_p70 rs1)
    (step-at-station black_cap_p70 cs2)
    (step-at-station gate2_delivery_p70 ds)
    (step-precedes silver_base_p70 blue_ring_p70)
    (step-precedes blue_ring_p70 orange_ring_p70)
    (step-precedes orange_ring_p70 yellow_ring_p70)
    (step-precedes yellow_ring_p70 black_cap_p70)
    (step-precedes black_cap_p70 gate2_delivery_p70)
    (material-required blue_ring_p70 one)
    (material-required orange_ring_p70 zero)
    (material-required yellow_ring_p70 two)
    (material-stored rs2 zero)
    (material-stored rs1 zero)
    (conveyor-empty rs1)
    (conveyor-empty rs2)
    (conveyor-empty cs1)
    (conveyor-empty cs2)
    (conveyor-empty bs)
    (conveyor-empty ds)
    (station-idle rs1)
    (station-idle rs2)
    (station-idle cs1)
    (station-idle cs2)
    (station-idle bs)
    (station-idle ds)
    (robot-idle r1)
    (robot-idle r2)
    (robot-idle r3)
    (robot-can-move r1)
    (robot-can-move r2)
    (robot-can-move r3)
    (robot-gripper-free r1)
    (robot-gripper-free r2)
    (robot-gripper-free r3)
    (step-incomplete silver_base_p10)
    (step-incomplete grey_cap_p10)
    (step-incomplete gate2_delivery_p10)
    (step-incomplete silver_base_p70)
    (step-incomplete blue_ring_p70)
    (step-incomplete orange_ring_p70)
    (step-incomplete yellow_ring_p70)
    (step-incomplete black_cap_p70)
    (step-incomplete gate2_delivery_p70)
    (cap-buffer-empty cs1)
    (cap-buffer-empty cs2)
    (location-free start)
    (location-free bs_in)
    (location-free bs_out)
    (location-free cs1_in)
    (location-free cs1_out)
    (location-free cs2_in)
    (location-free cs2_out)
    (location-free rs1_in)
    (location-free rs1_out)
    (location-free rs2_in)
    (location-free rs2_out)
    (location-free ds_in)
    (= (path-length bs_in bs_out) 9.80499)
    (= (path-length bs_in cs1_in) 56.2377)
    (= (path-length bs_in cs1_out) 63.8106)
    (= (path-length bs_in cs2_in) 50.0478)
    (= (path-length bs_in cs2_out) 39.3321)
    (= (path-length bs_in ds_in) 39.1852)
    (= (path-length bs_in rs1_in) 19.127)
    (= (path-length bs_in rs1_out) 30.0413)
    (= (path-length bs_in rs2_in) 21.2672)
    (= (path-length bs_in rs2_out) 11.5575)
    (= (path-length bs_out bs_in) 9.80499)
    (= (path-length bs_out cs1_in) 61.5629)
    (= (path-length bs_out cs1_out) 69.1358)
    (= (path-length bs_out cs2_in) 55.373)
    (= (path-length bs_out cs2_out) 44.6572)
    (= (path-length bs_out ds_in) 44.5104)
    (= (path-length bs_out rs1_in) 24.4522)
    (= (path-length bs_out rs1_out) 35.3664)
    (= (path-length bs_out rs2_in) 28.662)
    (= (path-length bs_out rs2_out) 15.5836)
    (= (path-length cs1_in bs_in) 56.2377)
    (= (path-length cs1_in bs_out) 61.5629)
    (= (path-length cs1_in cs1_out) 12.9856)
    (= (path-length cs1_in cs2_in) 21.4872)
    (= (path-length cs1_in cs2_out) 20.915)
    (= (path-length cs1_in ds_in) 26.9309)
    (= (path-length cs1_in rs1_in) 41.8211)
    (= (path-length cs1_in rs1_out) 31.225)
    (= (path-length cs1_in rs2_in) 47.0858)
    (= (path-length cs1_in rs2_out) 58.4958)
    (= (path-length cs1_out bs_in) 63.8106)
    (= (path-length cs1_out bs_out) 69.1358)
    (= (path-length cs1_out cs1_in) 12.9856)
    (= (path-length cs1_out cs2_in) 21.0577)
    (= (path-length cs1_out cs2_out) 29.0468)
    (= (path-length cs1_out ds_in) 36.2971)
    (= (path-length cs1_out rs1_in) 48.9831)
    (= (path-length cs1_out rs1_out) 39.3569)
    (= (path-length cs1_out rs2_in) 55.2177)
    (= (path-length cs1_out rs2_out) 66.6277)
    (= (path-length cs2_in bs_in) 50.0478)
    (= (path-length cs2_in bs_out) 55.373)
    (= (path-length cs2_in cs1_in) 21.4872)
    (= (path-length cs2_in cs1_out) 21.0577)
    (= (path-length cs2_in cs2_out) 23.8494)
    (= (path-length cs2_in ds_in) 32.989)
    (= (path-length cs2_in rs1_in) 35.2204)
    (= (path-length cs2_in rs1_out) 35.3077)
    (= (path-length cs2_in rs2_in) 51.1685)
    (= (path-length cs2_in rs2_out) 59.6831)
    (= (path-length cs2_out bs_in) 39.3321)
    (= (path-length cs2_out bs_out) 44.6572)
    (= (path-length cs2_out cs1_in) 20.915)
    (= (path-length cs2_out cs1_out) 29.0468)
    (= (path-length cs2_out cs2_in) 23.8494)
    (= (path-length cs2_out ds_in) 25.2384)
    (= (path-length cs2_out rs1_in) 24.5046)
    (= (path-length cs2_out rs1_out) 25.8839)
    (= (path-length cs2_out rs2_in) 41.7447)
    (= (path-length cs2_out rs2_out) 48.9673)
    (= (path-length ds_in bs_in) 39.1852)
    (= (path-length ds_in bs_out) 44.5104)
    (= (path-length ds_in cs1_in) 26.9309)
    (= (path-length ds_in cs1_out) 36.2971)
    (= (path-length ds_in cs2_in) 32.989)
    (= (path-length ds_in cs2_out) 25.2384)
    (= (path-length ds_in rs1_in) 25.4461)
    (= (path-length ds_in rs1_out) 14.1725)
    (= (path-length ds_in rs2_in) 29.2344)
    (= (path-length ds_in rs2_out) 40.6444)
    (= (path-length rs1_in bs_in) 19.127)
    (= (path-length rs1_in bs_out) 24.4522)
    (= (path-length rs1_in cs1_in) 41.8211)
    (= (path-length rs1_in cs1_out) 48.9831)
    (= (path-length rs1_in cs2_in) 35.2204)
    (= (path-length rs1_in cs2_out) 24.5046)
    (= (path-length rs1_in ds_in) 25.4461)
    (= (path-length rs1_in rs1_out) 26.0916)
    (= (path-length rs1_in rs2_in) 32.1897)
    (= (path-length rs1_in rs2_out) 28.7623)
    (= (path-length rs1_out bs_in) 30.0413)
    (= (path-length rs1_out bs_out) 35.3664)
    (= (path-length rs1_out cs1_in) 31.225)
    (= (path-length rs1_out cs1_out) 39.3569)
    (= (path-length rs1_out cs2_in) 35.3077)
    (= (path-length rs1_out cs2_out) 25.8839)
    (= (path-length rs1_out ds_in) 14.1725)
    (= (path-length rs1_out rs1_in) 26.0916)
    (= (path-length rs1_out rs2_in) 20.8894)
    (= (path-length rs1_out rs2_out) 32.2994)
    (= (path-length rs2_in bs_in) 21.2672)
    (= (path-length rs2_in bs_out) 28.662)
    (= (path-length rs2_in cs1_in) 47.0858)
    (= (path-length rs2_in cs1_out) 55.2177)
    (= (path-length rs2_in cs2_in) 51.1685)
    (= (path-length rs2_in cs2_out) 41.7447)
    (= (path-length rs2_in ds_in) 29.2344)
    (= (path-length rs2_in rs1_in) 32.1897)
    (= (path-length rs2_in rs1_out) 20.8894)
    (= (path-length rs2_in rs2_out) 14.2613)
    (= (path-length rs2_out bs_in) 11.5575)
    (= (path-length rs2_out bs_out) 15.5836)
    (= (path-length rs2_out cs1_in) 58.4958)
    (= (path-length rs2_out cs1_out) 66.6277)
    (= (path-length rs2_out cs2_in) 59.6831)
    (= (path-length rs2_out cs2_out) 48.9673)
    (= (path-length rs2_out ds_in) 40.6444)
    (= (path-length rs2_out rs1_in) 28.7623)
    (= (path-length rs2_out rs1_out) 32.2994)
    (= (path-length rs2_out rs2_in) 14.2613)
    (= (path-length start bs_in) 7.3014)
    (= (path-length start bs_out) 12.2342)
    (= (path-length start cs1_in) 53.621)
    (= (path-length start cs1_out) 61.1939)
    (= (path-length start cs2_in) 47.4312)
    (= (path-length start cs2_out) 36.7154)
    (= (path-length start ds_in) 36.5685)
    (= (path-length start rs1_in) 16.5103)
    (= (path-length start rs1_out) 27.4246)
    (= (path-length start rs2_in) 26.6464)
    (= (path-length start rs2_out) 16.9367)

    (robot-at-init r1 )
    ;(robot-at-init r2 )
    ;(robot-at-init r3 )
)
(:goal (and
    ;(step-completed silver_base_p10)
    ;(step-completed grey_cap_p10)
    (step-completed gate2_delivery_p10)
    
    ;(step-completed silver_base_p70)
    ;(step-completed blue_ring_p70)
    ;(step-completed orange_ring_p70)
    ;(step-completed yellow_ring_p70)
    ;(step-completed black_cap_p70)
    ;(step-completed gate2_delivery_p70)
    
    ;(robot-holding-product r1 p10)
    ;(station-prepared-for-step cs1 grey_cap_p10)
    ;(station-prepared-for-step ds gate2_delivery_p10)
    ;(robot-at r1 cs1_in)
    ;(cap-buffered cs1)
    ;(cap-buffered cs2)
    ;(material-at bs_in)
    ;(material-stored rs1 three)
    ;(material-stored rs2 three)
    ;(not (material-at cs2_out))
    ;(not (material-at cs1_out))

    (conveyor-empty bs)
    (conveyor-empty ds)
    (conveyor-empty cs1)
    (conveyor-empty cs2)
    (conveyor-empty rs1)
    (conveyor-empty rs2)
    (material-stored rs1 zero)
    (material-stored rs2 zero)
)))
