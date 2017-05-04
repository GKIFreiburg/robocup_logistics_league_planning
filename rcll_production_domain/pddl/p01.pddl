(define (problem p01)
(:domain rcll-production-steps)
(:objects
	r1 r2 r3 - robot
	p1 - product
	init_p1 ring1_p1 cap_p1 delivery_p1 - step
	p2 - product
	init_p2 ring1_p2 ring2_p2 ring3_p2 cap_p2 delivery_p2 - step
)

(:init
	; robots
	(robot-at-init r1)
	(robot-at-init r2)
	(robot-at-init r3)
	
	; p1
	(has-step p1 init_p1)
	(has-step p1 ring1_p1)
	(has-step p1 cap_p1)
	(has-step p1 delivery_p1)
	(step-at-machine init_p1 bs)
	(step-at-machine ring1_p1 rs1)
	(step-at-machine cap_p1 cs1)
	(step-at-machine delivery_p1 ds)
	(initial-step init_p1)
	(step-precedes init_p1 ring1_p1)
	(step-precedes ring1_p1 cap_p1)
	(step-precedes cap_p1 delivery_p1)
	(= (material-required ring1_p1) 2)

	; p2
	(has-step p2 init_p2)
	(has-step p2 ring1_p2)
	(has-step p2 ring2_p2)
	(has-step p2 ring3_p2)
	(has-step p2 cap_p2)
	(has-step p2 delivery_p2)
	(step-at-machine init_p2 bs)
	(step-at-machine ring1_p2 rs1)
	(step-at-machine ring2_p2 rs2)
	(step-at-machine ring3_p2 rs1)
	(step-at-machine cap_p2 cs2)
	(step-at-machine delivery_p2 ds)
	(initial-step init_p2)
	(step-precedes init_p2 ring1_p2)
	(step-precedes ring1_p2 ring2_p2)
	(step-precedes ring2_p2 ring3_p2)
	(step-precedes ring3_p2 cap_p2)
	(step-precedes cap_p2 delivery_p2)
	(= (material-required ring1_p2) 2)
	(= (material-required ring2_p2) 1)
	(= (material-required ring3_p2) 3)

	; stations
	(= (material-load rs1) 0)
	(= (material-load rs2) 0)
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
)

(:goal 
	(and 
		(step-completed delivery_p2)
		(step-completed delivery_p1)
		(forall (?l - location) (not (location-full ?l)))
		(= (material-load rs1) 0)
		(= (material-load rs2) 0)
	)  
)

(:metric minimize (total-time))

)
