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

	(:durative-action insert-cap
		:parameters (?r - robot ?m - cap_station ?i - cs_input)
		:duration (= ?duration 30)
		:condition (and
			(over all (idle ?m))
			(over all (robot-at ?r ?i))
			(at start (robot-idle ?r))
			(at start (robot-gripper-free ?r))
			(at start (input-location ?i ?m))
			(at start (conveyor-empty ?m))
			(at start (cap-buffer-empty ?m))
		)
		:effect (and
			(at start (not (conveyor-empty ?m)))
			(at start (not (robot-idle ?r)))
			(at end (robot-idle ?r))
			(at end (material-at ?i))
			(at end (robot-can-move ?r))
		)
	)

	(:durative-action drop-material
		:parameters (?r - robot ?o - cs_output ?om - cap_station)
		:duration (= ?duration 15)
		:condition (and
			(at start (robot-idle ?r))
			(at start (output-location ?o ?om))
			(at start (robot-at ?r ?o))
			(at start (material-at ?o))
			(at start (robot-gripper-free ?r))
		)
		:effect (and
			(at start (not (robot-idle ?r)))
			(at start (not (material-at ?o)))
			(at start (conveyor-empty ?om))
			(at end (not (robot-holding-material ?r)))
			(at end (robot-gripper-free ?r))
			(at end (robot-idle ?r))
			(at end (robot-can-move ?r))
		)
	)

	(:durative-action transport-material
		:parameters (?r - robot ?o - output ?om - machine ?i - rs_input ?m - ring_station ?mi ?mf - material_counter)
		:duration (= ?duration (+ 30 (path-length ?o ?i)))
		:condition (and
			(at start (robot-idle ?r))
			(at start (input-location ?i ?m))
			(at start (output-location ?o ?om))
			(at start (robot-at ?r ?o))
			(at start (material-at ?o))
			(at start (robot-gripper-free ?r))
			(over all (location-free ?i))
			(over all (material-stored ?m ?mi))
			(over all (add-one ?mi ?mf))
		)
		:effect (and
			(at start (not (robot-idle ?r)))
			(at start (not (robot-at ?r ?o)))
			(at start (not (material-at ?o)))
			(at start (conveyor-empty ?om))
			(at start (location-free ?o))
			(at start (robot-holding-material ?r))
			(at start (not (robot-gripper-free ?r)))
			(at end (not (robot-holding-material ?r)))
			(at end (robot-gripper-free ?r))
			(at end (not (location-free ?i)))
			(at end (robot-idle ?r))
			(at end (robot-at ?r ?i))
			(at end (robot-can-move ?r))
			;(at end (increase (material-stored ?m) 1))
			(at end (not (material-stored ?m ?mi)))
			(at end (material-stored ?m ?mf))
		)
	)

	(:durative-action transport-product
		:parameters (?r - robot ?p - product ?o - output ?om - machine ?i - input ?m - machine ?s1 ?s2 - step)
		:duration (= ?duration (+ 30 (path-length ?o ?i)))
		:condition (and
			(at start (robot-idle ?r))
			(at start (product-at ?p ?o))
			(at start (has-step ?p ?s1))
			(at start (has-step ?p ?s2))
			(at start (step-at-machine ?s2 ?m))
			(at start (step-precedes ?s1 ?s2))
			(at start (step-completed ?s1))
			(at start (step-incomplete ?s2))
			(at start (input-location ?i ?m))
			(at start (output-location ?o ?om))
			(at start (robot-at ?r ?o))
			(at start (robot-gripper-free ?r))
			(over all (location-free ?i))
			(over all (conveyor-empty ?m))
			;(over all (>= (material-stored ?m) (material-required ?s2)))
		)
		:effect (and
			(at start (not (robot-at ?r ?o)))
			(at start (not (robot-idle ?r)))
			(at start (not (product-at ?p ?o)))
			(at start (conveyor-empty ?om))
			(at start (location-free ?o))
			(at start (robot-holding-product ?r ?p))
			(at start (not (robot-gripper-free ?r)))
			(at end (not (robot-holding-product ?r ?p)))
			(at end (robot-gripper-free ?r))
			(at end (not (location-free ?i)))
			(at end (robot-idle ?r))
			(at end (robot-at ?r ?i))
			(at end (robot-can-move ?r))
			(at end (product-at ?p ?i))
			(at end (not (conveyor-empty ?m)))
		)
	)

	(:durative-action move
		:parameters (?r - robot ?l1 ?l2 - location)
		:duration (= ?duration (path-length ?l1 ?l2))
		:condition (and
			(at start (robot-idle ?r))
			(at start (robot-at ?r ?l1))
			(at start (robot-gripper-free ?r))
			(at start (robot-can-move ?r))
			(over all (location-free ?l2))
		)
		:effect (and
			(at start (not (robot-at ?r ?l1)))
			(at start (not (robot-idle ?r)))
			(at start (location-free ?l1))
			(at end (not (location-free ?l2)))
			(at end (robot-idle ?r))
			(at end (robot-at ?r ?l2))
			(at end (not (robot-can-move ?r)))
		)
	)

	(:durative-action move-in-first
		:parameters (?r - robot ?l - s_location)
		:duration (= ?duration 10)
		:condition (and
			(at start (robot-idle ?r))
			(at start (robot-at-init ?r))
			(at start (first-robot ?r))
			(over all (location-free ?l))
		)
		:effect (and
			(at start (not (robot-idle ?r)))
			(at end (not (robot-at-init ?r)))
			(at end (robot-not-at-init ?r))
			(at end (robot-idle ?r))
			(at end (not (location-free ?l)))
			(at end (robot-at ?r ?l))
		)
	)

	(:durative-action move-in
		:parameters (?r ?r2 - robot ?l - s_location)
		:duration (= ?duration 10)
		:condition (and
			(at start (robot-idle ?r))
			(at start (robot-at-init ?r))
			(at start (robot-precedes ?r2 ?r))
			(at start (robot-not-at-init ?r2))
			(over all (location-free ?l))
		)
		:effect (and
			(at end (not (robot-at-init ?r)))
			(at start (not (robot-idle ?r)))
			(at end (robot-idle ?r))
			(at end (not (location-free ?l)))
			(at end (robot-at ?r ?l))
		)
	)
)

