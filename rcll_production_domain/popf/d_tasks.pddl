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
		s_location - location
		station - object
		base_station ring_station cap_station delivery_station - station
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
		bs_in - output
		bs_out - output
		rs1_in - input
		rs1_out - output
		rs2_in - input
		rs2_out - output
		cs1_in - input
		cs1_out - output
		cs2_in - input
		cs2_out - output
		ds_in - input

		; materials
		zero one two three - material_counter
	)

	(:predicates
		; stations
		(input-location ?il - input ?m - station)
		(output-location ?ol - output ?m - station)
		(conveyor-empty ?m - station)
		
		; station states
		(station-idle ?m - station)
		(station-maintanence ?m - station)
		(station-output-ready ?m - station)
		(station-prepared-for-step ?m - station ?s - step)
		(station-prepared-for-dispense ?m - station)
		(prepared-dispense-output ?o - output)
		(station-prepared-for-cap ?m - cap_station)
		(station-prepared-for-discard ?m - delivery_station)
		
		; cap_station stations
		(cap-buffered ?m - cap_station)
		(cap-buffer-empty ?m - cap_station)
		
		; ring stations
		(material-required ?s - step ?r - material_counter)
		(material-stored ?m - station ?c - material_counter)
		(subtract ?minuend ?subtrahend ?difference - material_counter)
		(add-one ?summand ?sum - material_counter)
		(less-or-equal ?c1 ?c2 - material_counter)

		; steps
		(has-step ?p - product ?s - step)
		(step-incomplete ?s - step)
		(step-completed ?s - step)
		(initial-step ?s - step)
		(step-precedes ?s1 ?s2 - step)
		(step-at-station ?s - step ?m - station)
		
		; products
		(product-at ?p - product ?l - location)
		
		; materials
		(material-at ?l - location)
		
		; robots
		(robot-idle ?r - robot)
		(robot-outside ?r - robot)
		(robot-at ?r - robot ?l - location)
		(robot-holding-material ?r - robot)
		(robot-holding-product ?r - robot ?p - product)
		(robot-gripper-free ?r - robot)
	)

	(:functions
		; paths
		(path-length ?l1 ?l2 - location) - number
	)

	(:durative-action dispense-material
		:parameters (?m - base_station ?o - output)
		:duration (= ?duration 1)
		:condition (and
			(at start (conveyor-empty ?m))
			(at start (station-idle ?m))
		)
		:effect (and
			(at start (not (station-idle ?m)))
			(at end (station-idle ?m))
			(at start (not (conveyor-empty ?m)))
			(at end (material-at ?o))
		)
	)

	(:durative-action dispense-product
		:parameters (?p - product ?s - step ?m - base_station ?o - output)
		:duration (= ?duration 1)
		:condition (and
			(at start (has-step ?p ?s))
			(at start (step-at-station ?s ?m))
			(at start (initial-step ?s))
			(at start (step-incomplete ?s))
			(at start (conveyor-empty ?m))
			(at start (station-idle ?m))
		)
		:effect (and
			(at start (not (station-idle ?m)))
			(at end (station-idle ?m))
			(at start (not (conveyor-empty ?m)))
			(at end (product-at ?p ?o))
			(at end (not (step-incomplete ?s)))
			(at end (step-completed ?s))
		)
	)

	(:durative-action mount-ring
		:parameters (?m - ring_station ?p - product ?s1 ?s - step ?i - input ?o - output ?mi ?mr ?mf - material_counter)
		:duration (= ?duration 1)
		:condition (and
			(at start (product-at ?p ?i))
			(at start (has-step ?p ?s))
			(at start (step-at-station ?s ?m))
			(at start (step-incomplete ?s))
			(at start (step-completed ?s1))
			(at start (step-precedes ?s1 ?s))
			(at start (input-location ?i ?m))
			(at start (output-location ?o ?m))
			(at start (station-idle ?m))
			(at start (material-required ?s ?mr))
			(at start (material-stored ?m ?mi))
			(at start (subtract ?mi ?mr ?mf))
		)
		:effect (and
			(at start (not (station-idle ?m)))
			(at start (not (product-at ?p ?i)))
			(at start (not (material-stored ?m ?mi)))
			(at end (material-stored ?m ?mf))
			(at end (station-idle ?m))
			(at end (product-at ?p ?o))
			(at end (not (step-incomplete ?s)))
			(at end (step-completed ?s))
		)
	)

	(:durative-action buffer-cap
		:parameters (?m - cap_station ?i - input ?o - output)
		:duration (= ?duration 1)
		:condition (and
			(at start (input-location ?i ?m))
			(at start (output-location ?o ?m))
			(at start (material-at ?i))
			(at start (station-idle ?m))
			(at start (cap-buffer-empty ?m))
		)
		:effect (and
			(at start (not (station-idle ?m)))
			(at start (not (material-at ?i)))
			(at end (station-idle ?m))
			(at end (material-at ?o))
			(at end (not (cap-buffer-empty ?m)))
			(at end (cap-buffered ?m))
		)
	)

	(:durative-action mount-cap
		:parameters (?m - cap_station ?p - product ?s - step ?i - input ?o - output)
		:duration (= ?duration 1)
		:condition (and
			(at start (product-at ?p ?i))
			(at start (has-step ?p ?s))
			(at start (step-at-station ?s ?m))
			(at start (step-incomplete ?s))
			(at start (input-location ?i ?m))
			(at start (output-location ?o ?m))
			(at start (station-idle ?m))
			(at start (cap-buffered ?m))
		)
		:effect (and
			(at start (not (station-idle ?m)))
			(at start (not (product-at ?p ?i)))
			(at end (station-idle ?m))
			(at end (product-at ?p ?o))
			(at end (not (cap-buffered ?m)))
			(at end (cap-buffer-empty ?m))
			(at end (step-completed ?s))
			(at end (not (step-incomplete ?s)))
		)
	)

	(:durative-action deliver
		:parameters (?m - delivery_station ?p - product ?s - step ?i - input)
		:duration (= ?duration 1)
		:condition (and
			(at start (product-at ?p ?i))
			(at start (has-step ?p ?s))
			(at start (step-at-station ?s ?m))
			(at start (step-incomplete ?s))
			(at start (input-location ?i ?m))
			(at start (station-idle ?m))
		)
		:effect (and
			(at start (not (station-idle ?m)))
			(at start (not (product-at ?p ?i)))
			(at end (station-idle ?m))
			(at end (conveyor-empty ?m))
			(at end (step-completed ?s))
			(at end (not (step-incomplete ?s)))
		)
	)

	(:durative-action discard
		:parameters (?m - delivery_station ?i - input)
		:duration (= ?duration 1)
		:condition (and
			(at start (material-at ?i))
			(at start (input-location ?i ?m))
			(at start (station-idle ?m))
		)
		:effect (and
			(at start (not (station-idle ?m)))
			(at start (not (material-at ?i)))
			(at end (station-idle ?m))
			(at end (conveyor-empty ?m))
		)
	)

	(:durative-action discard-material
		:parameters (?m - delivery_station ?i - input)
		:duration (= ?duration 1)
		:condition (and
			(at start (material-at ?i))
			(at start (station-idle ?m))
			(at start (input-location ?i ?m))
		)
		:effect (and
			(at start (not (station-idle ?m)))
			(at end (station-idle ?m))
			(at end (not (material-at ?i)))
			(at end (conveyor-empty ?m))
		)
	)

	(:durative-action task-insert-cap
		:parameters (?m - cap_station ?i - input)
		:duration (= ?duration 30)
		:condition (and
			(over all (station-idle ?m))
			(at start (input-location ?i ?m))
			(at start (conveyor-empty ?m))
			(at start (cap-buffer-empty ?m))
		)
		:effect (and
			(at start (not (conveyor-empty ?m)))
			(at end (material-at ?i))
		)
	)

	(:durative-action task-transport-material-discard
		:parameters (?o - output ?om - station ?i - input ?m - delivery_station)
		:duration (= ?duration (+ 30 (path-length ?o ?i)))
		:condition (and
			(at start (input-location ?i ?m))
			(at start (output-location ?o ?om))
			(at start (material-at ?o))
			(over all (conveyor-empty ?m))
		)
		:effect (and
			(at start (not (material-at ?o)))
			(at start (conveyor-empty ?om))
			(at end (not (conveyor-empty ?m)))
			(at end (material-at ?i))
		)
	)

	(:durative-action task-transport-material
		:parameters (?o - output ?om - station ?i - input ?m - ring_station ?mi ?mf - material_counter)
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

	(:durative-action task-transport-product
		:parameters (?p - product ?o - output ?om - station ?i - input ?m - station ?s1 ?s2 - step)
		:duration (= ?duration (+ 30 (path-length ?o ?i)))
		:condition (and
			(at start (product-at ?p ?o))
			(at start (has-step ?p ?s1))
			(at start (has-step ?p ?s2))
			(at start (step-at-station ?s2 ?m))
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
