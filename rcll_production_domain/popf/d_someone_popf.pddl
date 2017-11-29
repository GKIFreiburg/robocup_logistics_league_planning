(define (domain rcll-production-steps)
	(:requirements 
		:durative-actions
		:numeric-fluents
		:object-fluents
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
		zero one two three - material_counter
	)

	(:predicates
		; stations
		(input-location ?il - input ?m - machine)
		(output-location ?ol - output ?m - machine)
		(idle ?m - machine)
		(conveyor-empty ?m - machine)
		
		; cap_station stations
		(cap-buffered ?m - cap_station)
		(cap-buffer-empty ?m - cap_station)
		
		; ring stations
		(material-required ?s - step ?r - material_counter)
		(material-stored ?m - machine ?r - material_counter)
		(subtract ?minuend ?subtrahend ?difference - material_counter)
		(add-one ?summand ?sum - material_counter)

		; steps
		(has-step ?p - product ?s - step)
		(step-incomplete ?s - step)
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
		(robot-not-at-init ?r - robot)
		(first-robot ?r - robot)
		(robot-precedes ?r1 ?r2 - robot)
		(robot-holding-material ?r - robot)
		(robot-holding-product ?r - robot ?p - product)
		(robot-gripper-free ?r - robot)
		(robot-can-move ?r - robot)
		(robot-idle ?r - robot)
		
		; locations
		(location-free ?l - location)
	)

	(:functions
		;(material-stored ?m - machine) - number
		;(material-required ?s - step) - number
		; paths
		(path-length ?l1 ?l2 - location) - number
	)

	(:durative-action dispense-material
		:parameters (?m - base_station ?o - bs_output)
		:duration (= ?duration 1)
		:condition (and
			(at start (conveyor-empty ?m))
			(at start (idle ?m))
		)
		:effect (and
			(at start (not (idle ?m)))
			(at end (idle ?m))
			(at start (not (conveyor-empty ?m)))
			(at end (material-at ?o))
		)
	)

	(:durative-action dispense-product
		:parameters (?p - product ?s - step ?m - base_station ?o - bs_output)
		:duration (= ?duration 1)
		:condition (and
			(at start (has-step ?p ?s))
			(at start (step-at-machine ?s ?m))
			(at start (initial-step ?s))
			(at start (step-incomplete ?s))
			(at start (conveyor-empty ?m))
			(at start (idle ?m))
		)
		:effect (and
			(at start (not (idle ?m)))
			(at end (idle ?m))
			(at start (not (conveyor-empty ?m)))
			(at end (product-at ?p ?o))
			(at end (not (step-incomplete ?s)))
			(at end (step-completed ?s))
		)
	)

	(:durative-action mount-ring
		:parameters (?m - ring_station ?p - product ?s1 ?s - step ?i - rs_input ?o - rs_output ?mi ?mr ?mf - material_counter)
		:duration (= ?duration 1)
		:condition (and
			(at start (product-at ?p ?i))
			(at start (has-step ?p ?s))
			(at start (step-at-machine ?s ?m))
			(at start (step-incomplete ?s))
			(at start (step-completed ?s1))
			(at start (step-precedes ?s1 ?s))
			(at start (input-location ?i ?m))
			(at start (output-location ?o ?m))
			(at start (idle ?m))
			;(at start (>= (material-stored ?m) (material-required ?s)))
			(at start (material-required ?s ?mr))
			(at start (material-stored ?m ?mi))
			(at start (subtract ?mi ?mr ?mf))
		)
		:effect (and
			(at start (not (idle ?m)))
			(at start (not (product-at ?p ?i)))
			(at end (idle ?m))
			(at end (product-at ?p ?o))
			(at end (not (step-incomplete ?s)))
			(at end (step-completed ?s))
			;(at end (decrease (material-stored ?m) (material-required ?s)))
			(at start (not (material-stored ?m ?mi)))
			(at start (material-stored ?m ?mf))
		)
	)

	(:durative-action buffer-cap
		:parameters (?m - cap_station ?i - cs_input ?o - cs_output)
		:duration (= ?duration 1)
		:condition (and
			(at start (input-location ?i ?m))
			(at start (output-location ?o ?m))
			(at start (material-at ?i))
			(at start (idle ?m))
			(at start (cap-buffer-empty ?m))
		)
		:effect (and
			(at start (not (idle ?m)))
			(at start (not (material-at ?i)))
			(at end (idle ?m))
			(at end (material-at ?o))
			(at end (not (cap-buffer-empty ?m)))
			(at end (cap-buffered ?m))
		)
	)

	(:durative-action mount-cap
		:parameters (?m - cap_station ?p - product ?s - step ?i - cs_input ?o - cs_output)
		:duration (= ?duration 1)
		:condition (and
			(at start (product-at ?p ?i))
			(at start (has-step ?p ?s))
			(at start (step-at-machine ?s ?m))
			(at start (step-incomplete ?s))
			(at start (input-location ?i ?m))
			(at start (output-location ?o ?m))
			(at start (idle ?m))
			(at start (cap-buffered ?m))
		)
		:effect (and
			(at start (not (idle ?m)))
			(at start (not (product-at ?p ?i)))
			(at end (idle ?m))
			(at end (product-at ?p ?o))
			(at end (not (cap-buffered ?m)))
			(at end (cap-buffer-empty ?m))
			(at end (step-completed ?s))
			(at end (not (step-incomplete ?s)))
		)
	)

	(:durative-action deliver
		:parameters (?m - delivery_station ?p - product ?s - step ?i - ds_input)
		:duration (= ?duration 1)
		:condition (and
			(at start (product-at ?p ?i))
			(at start (has-step ?p ?s))
			(at start (step-at-machine ?s ?m))
			(at start (step-incomplete ?s))
			(at start (idle ?m))
		)
		:effect (and
			(at start (not (idle ?m)))
			(at start (not (product-at ?p ?i)))
			(at end (idle ?m))
			(at end (conveyor-empty ?m))
			(at end (step-completed ?s))
			(at end (not (step-incomplete ?s)))
		)
	)

	(:durative-action discard-material
		:parameters (?m - delivery_station ?i - ds_input)
		:duration (= ?duration 1)
		:condition (and
			(at start (material-at ?i))
			(at start (idle ?m))
		)
		:effect (and
			(at start (not (idle ?m)))
			(at end (idle ?m))
			(at end (not (material-at ?i)))
			(at end (conveyor-empty ?m))
		)
	)

	(:durative-action someone-insert-cap
		:parameters (?m - cap_station ?i - cs_input)
		:duration (= ?duration 30)
		:condition (and
			(over all (idle ?m))
			(at start (input-location ?i ?m))
			(at start (conveyor-empty ?m))
			(at start (cap-buffer-empty ?m))
		)
		:effect (and
			(at start (not (conveyor-empty ?m)))
			(at end (material-at ?i))
		)
	)

	(:durative-action someone-drop-material
		:parameters (?o - cs_output ?om - cap_station)
		:duration (= ?duration 15)
		:condition (and
			(at start (output-location ?o ?om))
			(at start (material-at ?o))
		)
		:effect (and
			(at start (not (material-at ?o)))
			(at start (conveyor-empty ?om))
		)
	)

	(:durative-action someone-transport-material
		:parameters (?o - output ?om - machine ?i - rs_input ?m - ring_station ?mi ?mf - material_counter)
		:duration (= ?duration (+ 30 (path-length ?o ?i)))
		:condition (and
			(at start (input-location ?i ?m))
			(at start (output-location ?o ?om))
			(at start (material-at ?o))
			(over all (material-stored ?m ?mi))
			(over all (add-one ?mi ?mf))
		)
		:effect (and
			(at start (not (material-at ?o)))
			(at start (conveyor-empty ?om))
			(at end (not (material-stored ?m ?mi)))
			(at end (material-stored ?m ?mf))
		)
	)

	(:durative-action someone-transport-product
		:parameters (?p - product ?o - output ?om - machine ?i - input ?m - machine ?s1 ?s2 - step)
		:duration (= ?duration (+ 30 (path-length ?o ?i)))
		:condition (and
			(at start (product-at ?p ?o))
			(at start (has-step ?p ?s1))
			(at start (has-step ?p ?s2))
			(at start (step-at-machine ?s2 ?m))
			(at start (step-precedes ?s1 ?s2))
			(at start (step-completed ?s1))
			(at start (step-incomplete ?s2))
			(at start (input-location ?i ?m))
			(at start (output-location ?o ?om))
			(over all (conveyor-empty ?m))
		)
		:effect (and
			(at start (not (product-at ?p ?o)))
			(at start (conveyor-empty ?om))
			(at end (product-at ?p ?i))
			(at end (not (conveyor-empty ?m)))
		)
	)
)

