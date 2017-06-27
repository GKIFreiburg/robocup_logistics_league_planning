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
	(robot-at-init r1)
	(robot-at-init r2)
	(robot-at-init r3)
	(robot-precedes r1 r2)
	(robot-precedes r1 r3)
	(robot-precedes r2 r3)
	(robot-assigned-machine r1 cs1)
	(robot-assigned-machine r1 rs1)
	(robot-assigned-machine r2 cs2)
	(robot-assigned-machine r2 rs2)
	(robot-assigned-machine r1 bs)
	(robot-assigned-machine r2 bs)
	(robot-assigned-machine r3 bs)
	(step-at-machine silver_base_p10 bs)
	(step-at-machine grey_cap_p10 cs1)
	(step-at-machine gate2_delivery_p10 ds)
	(step-precedes silver_base_p10 grey_cap_p10)
	(step-precedes grey_cap_p10 gate2_delivery_p10)
	(step-at-machine silver_base_p70 bs)
	(step-at-machine blue_ring_p70 rs2)
	(step-at-machine orange_ring_p70 rs2)
	(step-at-machine yellow_ring_p70 rs1)
	(step-at-machine black_cap_p70 cs2)
	(step-at-machine gate2_delivery_p70 ds)
	;(step-completed silver_base_p70)
	;(step-completed orange_ring_p70)
	(step-precedes silver_base_p70 blue_ring_p70)
	(step-precedes blue_ring_p70 orange_ring_p70)
	(step-precedes orange_ring_p70 yellow_ring_p70)
	(step-precedes yellow_ring_p70 black_cap_p70)
	(step-precedes black_cap_p70 gate2_delivery_p70)
	(material-required silver_base_p10 zero)
	(material-required grey_cap_p10 zero)
	(material-required gate2_delivery_p10 zero)
	(material-required blue_ring_p70 one)
	(material-required orange_ring_p70 zero)
	(material-required yellow_ring_p70 two)
	(material-required silver_base_p70 zero)
	(material-required black_cap_p70 zero)
	(material-required gate2_delivery_p70 zero)
	(material-stored rs2 zero)
	(material-stored rs1 zero)
	(material-stored cs2 zero)
	(material-stored cs1 zero)
	(material-stored ds zero)
	(material-stored bs zero)
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
	(= (total-cost) 0)
)
(:goal (and
	;(step-completed silver_base_p10)
	(step-completed grey_cap_p10)
	;(step-completed gate2_delivery_p10)
	;(step-completed silver_base_p70)
	(step-completed blue_ring_p70)
	;(step-completed orange_ring_p70)
	;(step-completed yellow_ring_p70)
	;(step-completed black_cap_p70)
	;(step-completed gate2_delivery_p70)
	;(cap-buffered cs1)
	;(cap-buffered cs2)
	;(robot-at r1 bs_in)
	;(robot-holding-product r1 p70)
	;(robot-holding-material r1)
	;(not (material-at cs2_out))
	;(not (material-at cs1_out))
)))
