(define (domain rcll-production-steps)
	(:requirements 
		:typing
		:adl
	)

	(:types
		robot - object
		location - object
		input output - location
		ds_input rs_input cs_input - input
		bs_output rs_output cs_output - output
		s_location - location
		machine - object
		base_station ring_station cap_station delivery_station - machine
		product - object
		step - object
		material_counter - object
	)

	(:constants
		; stations
		bs - base_station
		rs1 - ring_station
		rs2 - ring_station
		cs1 - cap_station
		cs2 - cap_station
		ds - delivery_station

		; locations
		start - s_location
		bs_in - bs_output
		bs_out - bs_output
		rs1_in - rs_input
		rs1_out - rs_output
		rs2_in - rs_input
		rs2_out - rs_output
		cs1_in - cs_input
		cs1_out - cs_output
		cs2_in - cs_input
		cs2_out - cs_output
		ds_in - ds_input

		; materials
		m0 m1 m2 m3 - material_counter
	)

	(:predicates
		; stations
		(input-location ?il - input ?m - machine)
		(output-location ?ol - output ?m - machine)
		(conveyor-full ?m - machine)
		
		; ring stations
		(material-required ?s - step ?r - material_counter)
		(material-stored ?m - ring_station ?r - material_counter)
		(subtract ?minuend ?subtrahend ?difference - material_counter)
		(add-one ?summand ?sum - material_counter)

		; cap_station stations
		(cap-buffered ?m - cap_station)
		
		; steps
		(has-step ?p - product ?s - step)
		(step-completed ?s - step)
		(initial-step ?s - step)
		(step-precedes ?s1 ?s2 - step)
		(step-at-machine ?s - step ?m - machine)
		
		; products
		(product-at ?p - product ?l - location)
		
		; materials
		(material-at ?l - location)
		
		; robots
		(robot-at ?r - robot ?l - location)
		(robot-at-init ?r - robot)
		(robot-precedes ?r1 ?r2 - robot)
		(robot-holding-material ?r - robot)
		(robot-holding-product ?r - robot ?p - product)
		(robot-holding-something ?r - robot)
		(robot-recently-moved ?r - robot)
		(robot-assigned-machine ?r - robot ?m - machine)
		
		; locations
		(location-occupied ?l - location)
	)

	(:functions
		;(material-stored ?m - ring_station) - number
		;(material-required ?s - step) - number
		; paths
		(path-length ?l1 ?l2 - location) - number
		; cost
		(total-cost) - number
	)

	(:action dispense-material
		:parameters (?m - base_station ?o - bs_output)
		:condition (and
			(not (conveyor-full ?m))
		)
		:effect (and
			(conveyor-full ?m)
			(material-at ?o)
			(increase (total-cost) 1)
		)
	)

	(:action dispense-product
		:parameters (?p - product ?s - step ?m - base_station ?o - bs_output)
		:condition (and
			(has-step ?p ?s)
			(step-at-machine ?s ?m)
			(initial-step ?s)
			(not (step-completed ?s))
			(not (conveyor-full ?m))
		)
		:effect (and
			(conveyor-full ?m)
			(product-at ?p ?o)
			(step-completed ?s)
			(increase (total-cost) 1)
		)
	)

	(:action mount-ring
		:parameters (?m - ring_station ?p - product ?s1 ?s - step ?i - rs_input ?o - rs_output ?mi ?mr ?mf - material_counter)
		:condition (and
			(product-at ?p ?i)
			(has-step ?p ?s)
			(step-at-machine ?s ?m)
			(not (step-completed ?s))
			(step-completed ?s1)
			(step-precedes ?s1 ?s)
			(input-location ?i ?m)
			(output-location ?o ?m)
			;(>= (material-stored ?m) (material-required ?s))
			(material-required ?s ?mr)
			(material-stored ?m ?mi)
			(subtract ?mi ?mr ?mf)
		)
		:effect (and
			(not (product-at ?p ?i))
			(product-at ?p ?o)
			(step-completed ?s)
			;(decrease (material-stored ?m) (material-required ?s))
			(not (material-stored ?m ?mi))
			(material-stored ?m ?mf)
			(increase (total-cost) 1)
		)
	)

	(:action buffer-cap
		:parameters (?m - cap_station ?i - cs_input ?o - cs_output)
		:condition (and
			(input-location ?i ?m)
			(output-location ?o ?m)
			(material-at ?i)
			(not (cap-buffered ?m))
		)
		:effect (and
			(not (material-at ?i))
			(material-at ?o)
			(cap-buffered ?m)
			(increase (total-cost) 1)
		)
	)

	(:action mount-cap
		:parameters (?m - cap_station ?p - product ?s - step ?i - cs_input ?o - cs_output)
		:condition (and
			(product-at ?p ?i)
			(has-step ?p ?s)
			(step-at-machine ?s ?m)
			(not (step-completed ?s))
			(input-location ?i ?m)
			(output-location ?o ?m)
			(cap-buffered ?m)
		)
		:effect (and
			(not (product-at ?p ?i))
			(product-at ?p ?o)
			(not (cap-buffered ?m))
			(step-completed ?s)
			(increase (total-cost) 1)
		)
	)

	(:action deliver
		:parameters (?p - product ?s - step ?m - delivery_station ?i - ds_input)
		:condition (and
			(product-at ?p ?i)
			(has-step ?p ?s)
			(step-at-machine ?s ?m)
			(not (step-completed ?s))
		)
		:effect (and
			(not (product-at ?p ?i))
			(not (conveyor-full ?m))
			(step-completed ?s)
			(increase (total-cost) 1)
		)
	)

	(:action discard-material
		:parameters (?m - delivery_station ?i - ds_input)
		:condition (and
			(material-at ?i)
		)
		:effect (and
			(not (material-at ?i))
			(not (conveyor-full ?m))
			(increase (total-cost) 1)
		)
	)

	(:action insert-cap
		:parameters (?r - robot ?m - cap_station ?i - cs_input)
		:condition (and
			(robot-assigned-machine ?r ?m)
			(robot-at ?r ?i)
			(not (conveyor-full ?m))
			(not (robot-holding-something ?r))
			(input-location ?i ?m)
			(not (cap-buffered ?m))
		)
		:effect (and
			(conveyor-full ?m)
			(material-at ?i)
			(not (robot-recently-moved ?r))
			(increase (total-cost) 30)
		)
	)

	(:action pickup-material
		:parameters (?r - robot ?o - output ?m - machine)
		:condition (and
			(robot-assigned-machine ?r ?m)
			(robot-at ?r ?o)
			(not (robot-holding-something ?r))
			(output-location ?o ?m)
			(material-at ?o)
		)
		:effect (and
			(robot-holding-material ?r)
			(robot-holding-something ?r)
			(not (material-at ?o))
			(not (conveyor-full ?m))
			(not (robot-recently-moved ?r))
			(increase (total-cost) 15)
		)
	)

	(:action pickup-product
		:parameters (?r - robot ?o - output ?p - product ?m - machine)
		:condition (and
			(robot-assigned-machine ?r ?m)
			(robot-at ?r ?o)
			(output-location ?o ?m)
			(not (robot-holding-something ?r))
			(product-at ?p ?o)
		)
		:effect (and
			(robot-holding-product ?r ?p)
			(robot-holding-something ?r)
			(not (product-at ?p ?o))
			(not (conveyor-full ?m))
			(not (robot-recently-moved ?r))
			(increase (total-cost) 15)
		)
	)

	(:action insert-product
		:parameters (?r - robot ?i - input ?p - product ?m - machine)
		:condition (and
			(robot-at ?r ?i)
			(input-location ?i ?m)
			(robot-holding-product ?r ?p)
			(not (conveyor-full ?m))
		)
		:effect (and
			(not (robot-holding-product ?r ?p))
			(product-at ?p ?i)
			(conveyor-full ?m)
			(not (robot-recently-moved ?r))
			(not (robot-holding-something ?r))
			(increase (total-cost) 15)
		)
	)

	(:action insert-material
		:parameters (?r - robot ?i - rs_input ?m - ring_station ?mi ?mf - material_counter)
		:condition (and
			(robot-at ?r ?i)
			(input-location ?i ?m)
			(robot-holding-material ?r)
			;(< (material-stored ?m) 3)
			(material-stored ?m ?mi)
			(add-one ?mi ?mf)
		)
		:effect (and
			(not (robot-holding-material ?r))
			;(increase (material-stored ?m) 1)
			(not (material-stored ?m ?mi))
			(material-stored ?m ?mf)
			(not (robot-recently-moved ?r))
			(not (robot-holding-something ?r))
			(increase (total-cost) 15)
		)
	)

	(:action drop-material
		:parameters (?r - robot)
		:condition (and
			(robot-holding-material ?r)
		)
		:effect (and
			(not (robot-holding-material ?r))
			(not (robot-holding-something ?r))
			(not (robot-recently-moved ?r))
			(increase (total-cost) 1)
		)
	)

	(:action transport-material
		:parameters (?r - robot ?o - output ?i - rs_input ?m - ring_station)
		:condition (and
			(input-location ?i ?m)
			(robot-at ?r ?o)
			(not (robot-recently-moved ?r))
			(robot-holding-material ?r)
			(not (location-occupied ?i))
		)
		:effect (and
			(not (robot-at ?r ?o))
			(not (material-at ?o))
			(not (location-occupied ?o))
			(location-occupied ?i)
			(robot-at ?r ?i)
			(robot-recently-moved ?r)
			(increase (total-cost) (path-length ?o ?i))
		)
	)

	(:action transport-product
		:parameters (?r - robot ?p - product ?o - output ?i - input ?m - machine ?s1 ?s2 - step)
		:condition (and
			(robot-holding-product ?r ?p)
			(has-step ?p ?s1)
			(has-step ?p ?s2)
			(step-at-machine ?s2 ?m)
			(step-precedes ?s1 ?s2)
			(step-completed ?s1)
			(input-location ?i ?m)
			(not (step-completed ?s2))
			(robot-at ?r ?o)
			(not (robot-recently-moved ?r))
			(not (location-occupied ?i))
		)
		:effect (and
			(not (robot-at ?r ?o))
			(not (location-occupied ?o))
			(location-occupied ?i)
			(robot-at ?r ?i)
			(robot-recently-moved ?r)
			(increase (total-cost) (path-length ?o ?i))
		)
	)

	(:action move
		:parameters (?r - robot ?l1 ?l2 - location)
		:condition (and
			(robot-at ?r ?l1)
			(not (robot-holding-something ?r))
			(not (robot-recently-moved ?r))
			(not (location-occupied ?l2))
		)
		:effect (and
			(not (robot-at ?r ?l1))
			(not (location-occupied ?l1))
			(location-occupied ?l2)
			(robot-at ?r ?l2)
			(robot-recently-moved ?r)
			(increase (total-cost) (path-length ?l1 ?l2))
		)
	)

	(:action move-in
		:parameters (?r - robot ?l - s_location)
		:condition (and
			(robot-at-init ?r)
			(not (exists (?_r - robot) (and (robot-precedes ?_r ?r) (robot-at-init ?_r))))
			(not (location-occupied ?l))
		)
		:effect (and
			(not (robot-at-init ?r))
			(location-occupied ?l)
			(robot-at ?r ?l)
			(increase (total-cost) 10)
		)
	)
)

