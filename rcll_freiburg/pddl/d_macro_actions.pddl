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
	)

	(:predicates
		; stations
		(input-location ?il - input ?m - machine)
		(output-location ?ol - output ?m - machine)
		(processing ?m - machine)
		(conveyor-full ?m - machine)
		
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
		(robot-processing ?r - robot)
		(robot-assigned-machine ?r - robot ?m - machine)
		(robot-assigned-product-handling ?r - robot)
		
		; locations
		(location-occupied ?l - location)
	)

	(:functions
		(material-stored ?m - machine) - number
		(material-required ?s - step) - number
		; paths
		(path-length ?l1 ?l2 - location) - number
	)

	(:durative-action dispense-material
		:parameters (?m - base_station ?o - bs_output)
		:duration (= ?duration 1)
		:condition (and
			(at start (not (conveyor-full ?m)))
			(at start (not (processing ?m)))
		)
		:effect (and
			(at start (processing ?m))
			(at end (not (processing ?m)))
			(at start (conveyor-full ?m))
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
			(at start (not (step-completed ?s)))
			(at start (not (conveyor-full ?m)))
			(at start (not (processing ?m)))
		)
		:effect (and
			(at start (processing ?m))
			(at end (not (processing ?m)))
			(at start (conveyor-full ?m))
			(at end (product-at ?p ?o))
			(at end (step-completed ?s))
		)
	)

	(:durative-action mount-ring
		:parameters (?m - ring_station ?p - product ?s1 ?s - step ?i - rs_input ?o - rs_output)
		:duration (= ?duration 1)
		:condition (and
			(at start (product-at ?p ?i))
			(at start (has-step ?p ?s))
			(at start (step-at-machine ?s ?m))
			(at start (not (step-completed ?s)))
			(at start (step-completed ?s1))
			(at start (step-precedes ?s1 ?s))
			(at start (input-location ?i ?m))
			(at start (output-location ?o ?m))
			(at start (not (processing ?m)))
			(at start (>= (material-stored ?m) (material-required ?s)))
		)
		:effect (and
			(at start (processing ?m))
			(at start (not (product-at ?p ?i)))
			(at end (not (processing ?m)))
			(at end (product-at ?p ?o))
			(at end (step-completed ?s))
			(at end (decrease (material-stored ?m) (material-required ?s)))
		)
	)

	(:durative-action buffer-cap
		:parameters (?m - cap_station ?i - cs_input ?o - cs_output)
		:duration (= ?duration 1)
		:condition (and
			(at start (input-location ?i ?m))
			(at start (output-location ?o ?m))
			(at start (material-at ?i))
			(at start (not (processing ?m)))
			(at start (not (cap-buffered ?m)))
		)
		:effect (and
			(at start (processing ?m))
			(at start (not (material-at ?i)))
			(at end (not (processing ?m)))
			(at end (material-at ?o))
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
			(at start (not (step-completed ?s)))
			(at start (input-location ?i ?m))
			(at start (output-location ?o ?m))
			(at start (not (processing ?m)))
			(at start (cap-buffered ?m))
		)
		:effect (and
			(at start (processing ?m))
			(at start (not (product-at ?p ?i)))
			(at end (not (processing ?m)))
			(at end (product-at ?p ?o))
			(at end (not (cap-buffered ?m)))
			(at end (step-completed ?s))
		)
	)

	(:durative-action deliver
		:parameters (?m - delivery_station ?p - product ?s - step ?i - ds_input)
		:duration (= ?duration 1)
		:condition (and
			(at start (product-at ?p ?i))
			(at start (has-step ?p ?s))
			(at start (step-at-machine ?s ?m))
			(at start (not (step-completed ?s)))
			(at start (not (processing ?m)))
		)
		:effect (and
			(at start (processing ?m))
			(at start (not (product-at ?p ?i)))
			(at end (not (processing ?m)))
			(at end (not (conveyor-full ?m)))
			(at end (step-completed ?s))
		)
	)

	(:durative-action discard-material
		:parameters (?m - delivery_station ?i - ds_input)
		:duration (= ?duration 1)
		:condition (and
			(at start (material-at ?i))
			(at start (not (processing ?m)))
		)
		:effect (and
			(at start (processing ?m))
			(at end (not (processing ?m)))
			(at end (not (material-at ?i)))
			(at end (not (conveyor-full ?m)))
		)
	)

	(:durative-action insert-cap
		:parameters (?r - robot ?m - cap_station ?i - cs_input)
		:duration (= ?duration 30)
		:condition (and
			(over all (robot-assigned-machine ?r ?m))
			(over all (not (processing ?m)))
			(over all (robot-at ?r ?i))
			(at start (not (robot-processing ?r)))
			(at start (not (robot-holding-something ?r)))
			(at start (input-location ?i ?m))
			(at start (not (conveyor-full ?m)))
			(at start (not (cap-buffered ?m)))
		)
		:effect (and
			(at start (conveyor-full ?m))
			(at start (robot-processing ?r))
			(at end (not (robot-processing ?r)))
			(at end (material-at ?i))
			(at end (not (robot-recently-moved ?r)))
		)
	)

	(:durative-action drop-material
		:parameters (?r - robot ?o - cs_output ?om - cap_station)
		:duration (= ?duration 15)
		:condition (and
			(over all (robot-assigned-machine ?r ?om))
			(at start (not (robot-processing ?r)))
			(at start (output-location ?o ?om))
			(at start (robot-at ?r ?o))
			(at start (material-at ?o))
			(at start (not (robot-holding-something ?r)))
		)
		:effect (and
			(at start (robot-processing ?r))
			(at start (not (material-at ?o)))
			(at start (not (conveyor-full ?om)))
			(at end (not (robot-holding-material ?r)))
			(at end (not (robot-holding-something ?r)))
			(at end (not (robot-processing ?r)))
			(at end (not (robot-recently-moved ?r)))
		)
	)

	(:durative-action transport-material
		:parameters (?r - robot ?o - output ?om - machine ?i - rs_input ?m - ring_station)
		:duration (= ?duration (+ 30 (path-length ?o ?i)))
		:condition (and
			(at start (robot-assigned-machine ?r ?om))
			(at start (not (robot-processing ?r)))
			(at start (input-location ?i ?m))
			(at start (output-location ?o ?om))
			(at start (robot-at ?r ?o))
			(at start (material-at ?o))
			(at start (not (robot-holding-something ?r)))
			(over all (< (material-stored ?m) 3))
			(over all (not (location-occupied ?i)))
		)
		:effect (and
			(at start (robot-processing ?r))
			(at start (not (robot-at ?r ?o)))
			(at start (not (material-at ?o)))
			(at start (not (conveyor-full ?om)))
			(at start (not (location-occupied ?o)))
			(at start (robot-holding-material ?r))
			(at start (robot-holding-something ?r))
			(at end (not (robot-holding-material ?r)))
			(at end (not (robot-holding-something ?r)))
			(at end (location-occupied ?i))
			(at end (not (robot-processing ?r)))
			(at end (robot-at ?r ?i))
			(at end (not (robot-recently-moved ?r)))
			(at end (increase (material-stored ?m) 1))
		)
	)

	(:durative-action insert-material
		:parameters (?r - robot ?o - output ?om - machine ?i - rs_input ?m - ring_station)
		:duration (= ?duration (path-length ?o ?i))
		:condition (and
			(at start (not (robot-processing ?r)))
			(at start (input-location ?i ?m))
			(at start (output-location ?o ?om))
			(at start (robot-at ?r ?o))
			(at start (robot-holding-material ?r))
			(over all (< (material-stored ?m) 3))
			(over all (not (location-occupied ?i)))
		)
		:effect (and
			(at start (robot-processing ?r))
			(at start (not (robot-at ?r ?o)))
			(at start (not (location-occupied ?o)))
			(at end (not (robot-holding-material ?r)))
			(at end (not (robot-holding-something ?r)))
			(at end (location-occupied ?i))
			(at end (not (robot-processing ?r)))
			(at end (robot-at ?r ?i))
			(at end (not (robot-recently-moved ?r)))
			(at end (increase (material-stored ?m) 1))
		)
	)

	(:durative-action transport-product
		:parameters (?r - robot ?p - product ?o - output ?om - machine ?i - input ?m - machine ?s1 ?s2 - step)
		:duration (= ?duration (+ 30 (path-length ?o ?i)))
		:condition (and
			(at start (robot-assigned-machine ?r ?om))
			(at start (not (robot-processing ?r)))
			(at start (product-at ?p ?o))
			(at start (has-step ?p ?s1))
			(at start (has-step ?p ?s2))
			(at start (step-at-machine ?s2 ?m))
			(at start (step-precedes ?s1 ?s2))
			(at start (step-completed ?s1))
			(at start (not (step-completed ?s2)))
			(at start (input-location ?i ?m))
			(at start (output-location ?o ?om))
			(at start (robot-at ?r ?o))
			(at start (not (robot-holding-something ?r)))
			(over all (not (location-occupied ?i)))
			(over all (not (conveyor-full ?m)))
			(over all (>= (material-stored ?m) (material-required ?s2)))
		)
		:effect (and
			(at start (not (robot-at ?r ?o)))
			(at start (robot-processing ?r))
			(at start (not (product-at ?p ?o)))
			(at start (not (conveyor-full ?om)))
			(at start (not (location-occupied ?o)))
			(at start (robot-holding-product ?r ?p))
			(at start (robot-holding-something ?r))
			(at end (not (robot-holding-product ?r ?p)))
			(at end (not (robot-holding-something ?r)))
			(at end (location-occupied ?i))
			(at end (not (robot-processing ?r)))
			(at end (robot-at ?r ?i))
			(at end (not (robot-recently-moved ?r)))
			(at end (product-at ?p ?i))
			(at end (conveyor-full ?m))
		)
	)

	(:durative-action insert-product
		:parameters (?r - robot ?p - product ?o - output ?om - machine ?i - input ?m - machine ?s1 ?s2 - step)
		:duration (= ?duration (path-length ?o ?i))
		:condition (and
			(at start (not (robot-processing ?r)))
			(at start (robot-holding-product ?r ?p))
			(at start (has-step ?p ?s1))
			(at start (has-step ?p ?s2))
			(at start (step-at-machine ?s2 ?m))
			(at start (step-precedes ?s1 ?s2))
			(at start (step-completed ?s1))
			(at start (input-location ?i ?m))
			(at start (output-location ?o ?om))
			(at start (not (step-completed ?s2)))
			(at start (robot-at ?r ?o))
			(over all (not (location-occupied ?i)))
			(over all (not (conveyor-full ?m)))
			(over all (>= (material-stored ?m) (material-required ?s2)))
		)
		:effect (and
			(at start (not (robot-at ?r ?o)))
			(at start (robot-processing ?r))
			(at start (not (location-occupied ?o)))
			(at end (not (robot-holding-product ?r ?p)))
			(at end (not (robot-holding-something ?r)))
			(at end (location-occupied ?i))
			(at end (not (robot-processing ?r)))
			(at end (robot-at ?r ?i))
			(at end (not (robot-recently-moved ?r)))
			(at end (product-at ?p ?i))
			(at end (conveyor-full ?m))
		)
	)

	(:durative-action move
		:parameters (?r - robot ?l1 ?l2 - location)
		:duration (= ?duration (path-length ?l1 ?l2))
		:condition (and
			(at start (not (robot-processing ?r)))
			(at start (robot-at ?r ?l1))
			(at start (not (robot-holding-something ?r)))
			(at start (not (robot-recently-moved ?r)))
			(over all (not (location-occupied ?l2)))
		)
		:effect (and
			(at start (not (robot-at ?r ?l1)))
			(at start (robot-processing ?r))
			(at start (not (location-occupied ?l1)))
			(at end (location-occupied ?l2))
			(at end (not (robot-processing ?r)))
			(at end (robot-at ?r ?l2))
			(at end (robot-recently-moved ?r))
		)
	)

	(:durative-action move-in
		:parameters (?r - robot ?l - s_location)
		:duration (= ?duration 10)
		:condition (and
			(at start (not (robot-processing ?r)))
			(at start (robot-at-init ?r))
			(at start (not (exists (?r2 - robot) (and (robot-precedes ?r2 ?r) (robot-at-init ?r2)))))
			(over all (not (location-occupied ?l)))
		)
		:effect (and
			(at end (not (robot-at-init ?r)))
			(at start (robot-processing ?r))
			(at end (not (robot-processing ?r)))
			(at end (location-occupied ?l))
			(at end (robot-at ?r ?l))
		)
	)
)

