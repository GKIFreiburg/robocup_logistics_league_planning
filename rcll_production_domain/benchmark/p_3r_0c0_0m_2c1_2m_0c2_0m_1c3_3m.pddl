(define (problem p_3r_0c0_0m_2c1_2m_0c2_0m_1c3_3m)
(:domain rcll-production-steps)
(:objects
	; robots
	r1 r2 r3 - robot

	; products
	p1 - product
	init_p1  ring1_p1 cap_p1 delivery_p1 - step

	p2 - product
	init_p2  ring1_p2 cap_p2 delivery_p2 - step

	p3 - product
	init_p3  ring1_p3 ring2_p3 ring3_p3 cap_p3 delivery_p3 - step


)

(:init
	; robots
	(robot-at-init r1)
	(robot-at-init r2)
	(robot-at-init r3)

	(robot-precedes r1 r2)
	(robot-precedes r1 r3)
	(robot-precedes r2 r3)

	;p1
	(has-step p1 init_p1)
	(initial-step init_p1)
	(step-at-machine init_p1 bs)
	
	(has-step p1 ring1_p1)
	(step-precedes init_p1 ring1_p1)
	(step-at-machine ring1_p1 rs2)
	(= (material-required ring1_p1) 2)

	(has-step p1 cap_p1)
	(step-precedes  ring1_p1 cap_p1)
	(step-at-machine cap_p1 cs2)

	(has-step p1 delivery_p1)
	(step-precedes cap_p1 delivery_p1)
	(step-at-machine delivery_p1 ds)

	;p2
	(has-step p2 init_p2)
	(initial-step init_p2)
	(step-at-machine init_p2 bs)
	
	(has-step p2 ring1_p2)
	(step-precedes init_p2 ring1_p2)
	(step-at-machine ring1_p2 rs2)
	(= (material-required ring1_p2) 0)

	(has-step p2 cap_p2)
	(step-precedes  ring1_p2 cap_p2)
	(step-at-machine cap_p2 cs1)

	(has-step p2 delivery_p2)
	(step-precedes cap_p2 delivery_p2)
	(step-at-machine delivery_p2 ds)

	;p3
	(has-step p3 init_p3)
	(initial-step init_p3)
	(step-at-machine init_p3 bs)
	
	(has-step p3 ring1_p3)
	(step-precedes init_p3 ring1_p3)
	(step-at-machine ring1_p3 rs1)
	(= (material-required ring1_p3) 1)

	(has-step p3 ring2_p3)
	(step-precedes init_p3 ring2_p3)
	(step-at-machine ring2_p3 rs2)
	(= (material-required ring2_p3) 2)

	(has-step p3 ring3_p3)
	(step-precedes init_p3 ring3_p3)
	(step-at-machine ring3_p3 rs1)
	(= (material-required ring3_p3) 0)

	(has-step p3 cap_p3)
	(step-precedes  ring3_p3 cap_p3)
	(step-at-machine cap_p3 cs2)

	(has-step p3 delivery_p3)
	(step-precedes cap_p3 delivery_p3)
	(step-at-machine delivery_p3 ds)

	
	; stations
	(= (material-stored rs1) 0)
	(= (material-stored rs2) 0)
	(output-location bs_out bs)
	(input-location rs1_in rs1)
	(output-location rs1_out rs1)
	(input-location rs2_in rs2)
	(output-location rs2_out rs2)
	(input-location cs1_in cs1)
	(output-location cs1_out cs1)
	(input-location cs2_in cs2)
	(output-location cs2_out cs2)
	(input-location ds_in ds)

	; These values are for the static default world
;	(= (path-length bs_in bs_out) 9.80499)
;	(= (path-length bs_in cs1_in) 56.2377)
;	(= (path-length bs_in cs1_out) 63.8106)
;	(= (path-length bs_in cs2_in) 50.0478)
;	(= (path-length bs_in cs2_out) 39.3321)
;	(= (path-length bs_in ds_in) 39.1852)
;	(= (path-length bs_in ds_out) 27.5249)
;	(= (path-length bs_in rs1_in) 19.127)
;	(= (path-length bs_in rs1_out) 30.0413)
;	(= (path-length bs_in rs2_in) 21.2672)
;	(= (path-length bs_in rs2_out) 11.5575)
;	(= (path-length bs_in start) 7.3014)
;	(= (path-length bs_out bs_in) 9.80499)
	(= (path-length bs_out cs1_in) 61.5629)
	(= (path-length bs_out cs1_out) 69.1358)
	(= (path-length bs_out cs2_in) 55.373)
	(= (path-length bs_out cs2_out) 44.6572)
	(= (path-length bs_out ds_in) 44.5104)
;	(= (path-length bs_out ds_out) 34.9196)
	(= (path-length bs_out rs1_in) 24.4522)
	(= (path-length bs_out rs1_out) 35.3664)
	(= (path-length bs_out rs2_in) 28.662)
	(= (path-length bs_out rs2_out) 15.5836)
	(= (path-length bs_out start) 12.2342)
;	(= (path-length cs1_in bs_in) 56.2377)
	(= (path-length cs1_in bs_out) 61.5629)
	(= (path-length cs1_in cs1_out) 12.9856)
	(= (path-length cs1_in cs2_in) 21.4872)
	(= (path-length cs1_in cs2_out) 20.915)
	(= (path-length cs1_in ds_in) 26.9309)
;	(= (path-length cs1_in ds_out) 35.1973)
	(= (path-length cs1_in rs1_in) 41.8211)
	(= (path-length cs1_in rs1_out) 31.2251)
	(= (path-length cs1_in rs2_in) 47.0858)
	(= (path-length cs1_in rs2_out) 58.4958)
	(= (path-length cs1_in start) 53.621)
;	(= (path-length cs1_out bs_in) 63.8106)
	(= (path-length cs1_out bs_out) 69.1358)
	(= (path-length cs1_out cs1_in) 12.9856)
	(= (path-length cs1_out cs2_in) 21.0577)
	(= (path-length cs1_out cs2_out) 29.0468)
	(= (path-length cs1_out ds_in) 36.2971)
;	(= (path-length cs1_out ds_out) 44.5635)
	(= (path-length cs1_out rs1_in) 48.9831)
	(= (path-length cs1_out rs1_out) 39.3569)
	(= (path-length cs1_out rs2_in) 55.2177)
	(= (path-length cs1_out rs2_out) 66.6277)
	(= (path-length cs1_out start) 61.1939)
;	(= (path-length cs2_in bs_in) 50.0478)
	(= (path-length cs2_in bs_out) 55.373)
	(= (path-length cs2_in cs1_in) 21.4872)
	(= (path-length cs2_in cs1_out) 21.0577)
	(= (path-length cs2_in cs2_out) 23.8494)
	(= (path-length cs2_in ds_in) 32.989)
;	(= (path-length cs2_in ds_out) 41.2555)
	(= (path-length cs2_in rs1_in) 35.2204)
	(= (path-length cs2_in rs1_out) 35.3078)
	(= (path-length cs2_in rs2_in) 51.1685)
	(= (path-length cs2_in rs2_out) 59.6831)
	(= (path-length cs2_in start) 47.4312)
;	(= (path-length cs2_out bs_in) 39.3321)
	(= (path-length cs2_out bs_out) 44.6572)
	(= (path-length cs2_out cs1_in) 20.915)
	(= (path-length cs2_out cs1_out) 29.0468)
	(= (path-length cs2_out cs2_in) 23.8494)
	(= (path-length cs2_out ds_in) 25.2384)
;	(= (path-length cs2_out ds_out) 37.7419)
	(= (path-length cs2_out rs1_in) 24.5046)
	(= (path-length cs2_out rs1_out) 25.8839)
	(= (path-length cs2_out rs2_in) 41.7447)
	(= (path-length cs2_out rs2_out) 48.9673)
	(= (path-length cs2_out start) 36.7154)
;	(= (path-length ds_in bs_in) 39.1852)
	(= (path-length ds_in bs_out) 44.5104)
	(= (path-length ds_in cs1_in) 26.9309)
	(= (path-length ds_in cs1_out) 36.2971)
	(= (path-length ds_in cs2_in) 32.989)
	(= (path-length ds_in cs2_out) 25.2384)
;	(= (path-length ds_in ds_out) 14.639)
	(= (path-length ds_in rs1_in) 25.4461)
	(= (path-length ds_in rs1_out) 14.1725)
	(= (path-length ds_in rs2_in) 29.2344)
	(= (path-length ds_in rs2_out) 40.6444)
	(= (path-length ds_in start) 36.5685)
;	(= (path-length ds_out bs_in) 27.5249)
;	(= (path-length ds_out bs_out) 34.9196)
;	(= (path-length ds_out cs1_in) 35.1973)
;	(= (path-length ds_out cs1_out) 44.5635)
;	(= (path-length ds_out cs2_in) 41.2555)
;	(= (path-length ds_out cs2_out) 37.7419)
;	(= (path-length ds_out ds_in) 14.639)
;	(= (path-length ds_out rs1_in) 32.2671)
;	(= (path-length ds_out rs1_out) 20.9667)
;	(= (path-length ds_out rs2_in) 17.2738)
;	(= (path-length ds_out rs2_out) 28.6838)
;	(= (path-length ds_out start) 32.4234)
;	(= (path-length rs1_in bs_in) 19.127)
	(= (path-length rs1_in bs_out) 24.4522)
	(= (path-length rs1_in cs1_in) 41.8211)
	(= (path-length rs1_in cs1_out) 48.9831)
	(= (path-length rs1_in cs2_in) 35.2204)
	(= (path-length rs1_in cs2_out) 24.5046)
	(= (path-length rs1_in ds_in) 25.4461)
;	(= (path-length rs1_in ds_out) 32.2671)
	(= (path-length rs1_in rs1_out) 26.0916)
	(= (path-length rs1_in rs2_in) 32.1897)
	(= (path-length rs1_in rs2_out) 28.7623)
	(= (path-length rs1_in start) 16.5103)
;	(= (path-length rs1_out bs_in) 30.0413)
	(= (path-length rs1_out bs_out) 35.3664)
	(= (path-length rs1_out cs1_in) 31.225)
	(= (path-length rs1_out cs1_out) 39.3569)
	(= (path-length rs1_out cs2_in) 35.3077)
	(= (path-length rs1_out cs2_out) 25.8839)
	(= (path-length rs1_out ds_in) 14.1725)
;	(= (path-length rs1_out ds_out) 20.9667)
	(= (path-length rs1_out rs1_in) 26.0916)
	(= (path-length rs1_out rs2_in) 20.8894)
	(= (path-length rs1_out rs2_out) 32.2994)
	(= (path-length rs1_out start) 27.4246)
;	(= (path-length rs2_in bs_in) 21.2672)
	(= (path-length rs2_in bs_out) 28.662)
	(= (path-length rs2_in cs1_in) 47.0858)
	(= (path-length rs2_in cs1_out) 55.2177)
	(= (path-length rs2_in cs2_in) 51.1685)
	(= (path-length rs2_in cs2_out) 41.7447)
	(= (path-length rs2_in ds_in) 29.2344)
;	(= (path-length rs2_in ds_out) 17.2738)
	(= (path-length rs2_in rs1_in) 32.1897)
	(= (path-length rs2_in rs1_out) 20.8894)
	(= (path-length rs2_in rs2_out) 14.2613)
	(= (path-length rs2_in start) 26.6464)
;	(= (path-length rs2_out bs_in) 11.5575)
	(= (path-length rs2_out bs_out) 15.5836)
	(= (path-length rs2_out cs1_in) 58.4958)
	(= (path-length rs2_out cs1_out) 66.6277)
	(= (path-length rs2_out cs2_in) 59.6831)
	(= (path-length rs2_out cs2_out) 48.9673)
	(= (path-length rs2_out ds_in) 40.6444)
;	(= (path-length rs2_out ds_out) 28.6838)
	(= (path-length rs2_out rs1_in) 28.7623)
	(= (path-length rs2_out rs1_out) 32.2994)
	(= (path-length rs2_out rs2_in) 14.2613)
	(= (path-length rs2_out start) 16.9367)
;	(= (path-length start bs_in) 7.3014)
	(= (path-length start bs_out) 12.2342)
	(= (path-length start cs1_in) 53.621)
	(= (path-length start cs1_out) 61.1939)
	(= (path-length start cs2_in) 47.4312)
	(= (path-length start cs2_out) 36.7154)
	(= (path-length start ds_in) 36.5685)
;	(= (path-length start ds_out) 32.4234)
	(= (path-length start rs1_in) 16.5103)
	(= (path-length start rs1_out) 27.4246)
	(= (path-length start rs2_in) 26.6464)
	(= (path-length start rs2_out) 16.9367)
)

(:goal 
	(and 
		(forall (?_s - step)(step-completed ?_s))
	)  
)

(:metric minimize (total-time))

)
