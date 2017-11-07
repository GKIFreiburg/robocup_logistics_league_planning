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
		
		; locations
		(product-at ?p - product ?l - location)
		(material-at ?l - location)
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

	(:durative-action someone-discard-material
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

	(:durative-action someone-insert-cap
		:parameters (?m - cap_station ?i - cs_input)
		:duration (= ?duration 30)
		:condition (and
			(over all (not (processing ?m)))
			(at start (input-location ?i ?m))
			(at start (not (conveyor-full ?m)))
			(at start (not (cap-buffered ?m)))
		)
		:effect (and
			(at start (conveyor-full ?m))
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
			(at start (not (conveyor-full ?om)))
		)
	)

	(:durative-action someone-transport-material
		:parameters (?o - output ?om - machine ?i - rs_input ?m - ring_station)
		:duration (= ?duration (+ 30 (path-length ?o ?i)))
		:condition (and
			(at start (input-location ?i ?m))
			(at start (output-location ?o ?om))
			(at start (material-at ?o))
			(over all (< (material-stored ?m) 3))
		)
		:effect (and
			(at start (not (material-at ?o)))
			(at start (not (conveyor-full ?om)))
			(at end (increase (material-stored ?m) 1))
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
			(at start (not (step-completed ?s2)))
			(at start (input-location ?i ?m))
			(at start (output-location ?o ?om))
			(over all (not (conveyor-full ?m)))
			(over all (>= (material-stored ?m) (material-required ?s2)))
		)
		:effect (and
			(at start (not (product-at ?p ?o)))
			(at start (not (conveyor-full ?om)))
			(at end (product-at ?p ?i))
			(at end (conveyor-full ?m))
		)
	)
)

