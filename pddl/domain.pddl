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
		input material output - location
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
	)

	(:predicates
		; stations
		(input-location ?il - input ?m - machine)
		(output-location ?ol - output ?m - machine)
		(processing ?m - machine)
		
		; cap_station stations
		(cap-buffered ?m - cap_station)
		
		; steps
		(has-step ?p - product ?s - step)
		(step-completed ?s - step)
		(initial-step ?s1 - step)
		(step-precedes ?s1 ?s2 - step)
		(step-at-machine ?s - step ?m - machine)
		
		; locations
		(location-full ?l - location)
		
		; products
		(product-at ?p - product ?l - location)
		
		; materials
		(material-at ?l - location)
		
		; robots
		(robot-at ?r - robot ?l - location)
		(robot-at-init ?r)
	)

	(:functions
		(material-load ?m - ring_station) - number
		(material-required ?s - step) - number
	)

	(:durative-action initialize-material
		:parameters (?m - base_station ?o - output)
		:duration (= ?duration 1)
		:condition (and
			(at start (output-location ?o ?m))
			(at start (not (location-full ?o)))
			(at start (not (processing ?m)))
		)
		:effect (and
			(at start (processing ?m))
			(at end (not (processing ?m)))
			(at start (location-full ?o))
			(at end (material-at ?o))
		)
	)

	(:durative-action initialize-product
		:parameters (?m - base_station ?p - product ?s - step ?o - output)
		:duration (= ?duration 1)
		:condition (and
			(at start (has-step ?p ?s))
			(at start (step-at-machine ?s ?m))
			(at start (initial-step ?s))
			(at start (not (step-completed ?s)))
			(at start (output-location ?o ?m))
			(at start (not (location-full ?o)))
			(at start (not (processing ?m)))
		)
		:effect (and
			(at start (processing ?m))
			(at end (not (processing ?m)))
			(at start (location-full ?o))
			(at end (product-at ?p ?o))
			(at end (step-completed ?s))
		)
	)

	(:durative-action load-material
		:parameters (?m - ring_station ?s - step ?i - input ?o - output)
		:duration (= ?duration 1)
		:condition (and
			(at start (material-at ?i))
			(at start (step-at-machine ?s ?m))
			(at start (not (step-completed ?s)))
			(at start (input-location ?i ?m))
			(at start (output-location ?o ?m))
			(at start (not (location-full ?o)))
			(at start (not (processing ?m)))
			(at start (< (material-load ?m) (material-required ?s)))
		)
		:effect (and
			(at start (processing ?m))
			(at end (not (processing ?m)))
			(at end (not (material-at ?i)))
			(at end (not (location-full ?i)))
			(at end (increase (material-load ?m) 1))
		)
	)

	(:durative-action mount-ring
		:parameters (?m - ring_station ?p - product ?s - step ?i - input ?o - output)
		:duration (= ?duration 1)
		:condition (and
			(at start (product-at ?p ?i))
			(at start (has-step ?p ?s))
			(at start (step-at-machine ?s ?m))
			(at start (not (step-completed ?s)))
			(at start (input-location ?i ?m))
			(at start (output-location ?o ?m))
			(at start (not (location-full ?o)))
			(at start (not (processing ?m)))
			(at start (= (material-load ?m) (material-required ?s)))
		)
		:effect (and
			(at start (processing ?m))
			(at end (not (processing ?m)))
			(at start (not (product-at ?p ?i)))
			(at start (location-full ?o))
			(at end (not (location-full ?i)))
			(at end (not (processing ?m)))
			(at end (product-at ?p ?o))
			(at end (step-completed ?s))
			(at end (assign (material-load ?m) 0))
		)
	)

	(:durative-action buffer-cap
		:parameters (?r - robot ?m - cap_station ?i - input ?o - output)
		:duration (= ?duration 1)
		:condition (and
			(over all (robot-at ?r ?i))
			(at start (input-location ?i ?m))
			(at start (output-location ?o ?m))
			(at start (not (location-full ?i)))
			(at start (not (location-full ?o)))
			(at start (not (processing ?m)))
			(at start (not (cap-bufferd ?m)))
		)
		:effect (and
			(at start (processing ?m))
			(at start (location-full ?o))
			(at end (material-at ?o))
			(at end (not (processing ?m)))
			(at end (cap-bufferd ?m))
		)
	)

	(:durative-action mount-cap
		:parameters (?m - cap_station ?p - product ?s - step ?i - input ?o - output)
		:duration (= ?duration 1)
		:condition (and
			(at start (product-at ?p ?i))
			(at start (has-step ?p ?s))
			(at start (step-at-machine ?s ?m))
			(at start (not (step-completed ?s)))
			(at start (input-location ?i ?m))
			(at start (output-location ?o ?m))
			(at start (not (location-full ?o)))
			(at start (not (processing ?m)))
			(at start (cap-bufferd ?m))
		)
		:effect (and
			(at start (processing ?m))
			(at start (not (product-at ?p ?i)))
			(at start (location-full ?o))
			(at end (not (processing ?m)))
			(at end (not (cap-bufferd ?m)))
			(at end (not (location-full ?i)))
			(at end (not (processing ?m)))
			(at end (product-at ?p ?o))
			(at end (step-completed ?s))
		)
	)

	(:durative-action deliver
		:parameters (?m - delivery_station ?p - product ?s - step ?i - input)
		:duration (= ?duration 1)
		:condition (and
			(at start (product-at ?p ?i))
			(at start (has-step ?p ?s))
			(at start (step-at-machine ?s ?m))
			(at start (not (step-completed ?s)))
			(at start (input-location ?i ?m))
			(at start (not (processing ?m)))
		)
		:effect (and
			(at start (processing ?m))
			(at end (not (processing ?m)))
			(at start (not (product-at ?p ?i)))
			(at end (not (location-full ?i)))
			(at end (not (processing ?m)))
			(at end (step-completed ?s))
		)
	)

	(:durative-action discard-material
		:parameters (?m - delivery_station ?i - input)
		:duration (= ?duration 1)
		:condition (and
			(at start (material-at ?i))
			(at start (input-location ?i ?m))
			(at start (not (processing ?m)))
		)
		:effect (and
			(at start (processing ?m))
			(at end (not (processing ?m)))
			(at end (not (material-at ?i)))
			(at end (not (location-full ?i)))
		)
	)

	(:durative-action transport-material-discard
		:parameters (?r - robot ?o - output ?i - input ?m - delivery_station)
		:duration (= ?duration 1)
		:condition (and
			(at start (input-location ?i ?m))
			(at start (robot-at ?r ?o))
			(at start (material-at ?o))
			(at end (not (location-full ?i)))
		)
		:effect (and
			(at start (not (location-full ?o)))
			(at start (not (robot-at ?r ?o)))
			(at start (not (material-at ?o)))
			(at end (location-full ?i))
			(at end (robot-at ?r ?i))
			(at end (material-at ?i))
		)
	)

	(:durative-action transport-material
		:parameters (?r - robot ?o - output ?i - input ?m - ring_station)
		:duration (= ?duration 1)
		:condition (and
			(at start (input-location ?i ?m))
			(at start (robot-at ?r ?o))
			(at start (material-at ?o))
			(at end (not (location-full ?i)))
		)
		:effect (and
			(at start (not (location-full ?o)))
			(at start (not (robot-at ?r ?o)))
			(at start (not (material-at ?o)))
			(at end (location-full ?i))
			(at end (robot-at ?r ?i))
			(at end (material-at ?i))
		)
	)

	(:durative-action transport-product
		:parameters (?r - robot ?p - product ?o - output ?i - input ?m - machine ?s1 ?s2 - step)
		:duration (= ?duration 1)
		:condition (and
			(at start (product-at ?p ?o))
			(at start (has-step ?p ?s1))
			(at start (has-step ?p ?s2))
			(at start (step-at-machine ?s2 ?m))
			(at start (step-precedes ?s1 ?s2))
			(at start (step-completed ?s1))
			(at start (input-location ?i ?m))
			(at start (not (step-completed ?s2)))
			(at start (robot-at ?r ?o))
			(at end (not (location-full ?i)))
		)
		:effect (and
			(at start (not (location-full ?o)))
			(at start (not (robot-at ?r ?o)))
			(at start (not (product-at ?p ?o)))
			(at end (location-full ?i))
			(at end (robot-at ?r ?i))
			(at end (product-at ?p ?i))
		)
	)

	(:durative-action move
		:parameters (?r - robot ?l1 - output ?l2 - input)
		:duration (= ?duration 1)
		:condition (and
			(at start (robot-at ?r ?l1))
		)
		:effect (and
			(at start (not (robot-at ?r ?l1)))
			(at end (robot-at ?r ?l2))
		)
	)

	(:durative-action move-in
		:parameters (?r - robot ?l - location)
		:duration (= ?duration 1)
		:condition (and
			(at start (robot-at-init ?r))
		)
		:effect (and
			(at start (not (robot-at-init ?r)))
			(at end (robot-at ?r ?l))
		)
	)	
)

