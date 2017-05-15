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
		start - location
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
		(conveyor-full ?m - machine)
		
		; cap_station stations
		(cap-buffered ?m - cap_station)
		
		; steps
		(has-step ?p - product ?s - step)
		(step-completed ?s - step)
		(initial-step ?s1 - step)
		(step-precedes ?s1 ?s2 - step)
		(step-at-machine ?s - step ?m - machine)
		
		; products
		(product-at ?p - product ?l - location)
		(product-precedes ?p1 ?p2 - product)
		(product-active ?p - product)
		
		; materials
		(material-at ?l - location)
		
		; robots
		(robot-at ?r - robot ?l - location)
		(robot-at-init ?r - robot)
		(robot-precedes ?r1 ?r2 - robot)
		(robot-holding-material ?r - robot)
		(robot-holding-product ?r - robot ?p - product)
	)

	(:functions
		(material-load ?m - ring_station) - number
		(material-required ?s - step) - number
		; paths
		(path-length ?l1 ?l2 - location) - number
		; products
		(product-count) - number
		(max-product-count) - number
	)

	(:durative-action dispense-material
		:parameters ()
		:duration (= ?duration 15)
		:condition (and
			(at start (not (conveyor-full bs)))
			(at start (not (processing bs)))
		)
		:effect (and
			(at start (processing bs))
			(at end (not (processing bs)))
			(at end (conveyor-full bs))
			(at end (material-at bs_out))
		)
	)

	(:durative-action dispense-product
		:parameters (?p - product ?s - step)
		:duration (= ?duration 15)
		:condition (and
			(at start (has-step ?p ?s))
			(at start (step-at-machine ?s bs))
			(at start (initial-step ?s))
			(at start (not (step-completed ?s)))
			(at start (not (conveyor-full bs)))
			(at start (not (processing bs)))
			(at start (< (product-count) (max-product-count)))
			(at start (can-start-product ?p))
		)
		:effect (and
			(at start (processing bs))
			(at end (not (processing bs)))
			(at end (conveyor-full bs))
			(at end (product-at ?p bs_out))
			(at end (product-active ?p))
			(at end (step-completed ?s))
			(at end (increase (product-count) 1))
		)
	)

	(:durative-action mount-ring
		:parameters (?m - ring_station ?p - product ?s - step ?i - input ?o - output)
		:duration (= ?duration 60)
		:condition (and
			(at start (product-at ?p ?i))
			(at start (has-step ?p ?s))
			(at start (step-at-machine ?s ?m))
			(at start (not (step-completed ?s)))
			(at start (input-location ?i ?m))
			(at start (output-location ?o ?m))
			(at start (not (processing ?m)))
			(at start (>= (material-load ?m) (material-required ?s)))
		)
		:effect (and
			(at start (processing ?m))
			(at end (not (processing ?m)))
			(at start (not (product-at ?p ?i)))
			(at end (product-at ?p ?o))
			(at end (step-completed ?s))
			(at end (decrease (material-load ?m) (material-required ?s)))
		)
	)

	(:durative-action move-to-buffer
		:parameters (?r - robot ?l - location ?i - input ?m - cap_station)
		:duration (= ?duration (path-length ?l ?i))
		:condition (and
			(at start (robot-at ?r ?l))
			(at start (robot-can-pickup ?r))
			(at start (input-location ?i ?m))
			(at end (no-robot-at-location ?i))
			(at end (not (conveyor-full ?m)))
			(at end (not (cap-buffered ?m)))
		)
		:effect (and
			(at start (not (robot-at ?r ?l)))
			(at end (robot-at ?r ?i))
		)
	)

	(:durative-action buffer-cap
		:parameters (?r - robot ?m - cap_station ?i - input ?o - output)
		:duration (= ?duration 25)
		:condition (and
			(over all (robot-at ?r ?i))
			(at start (input-location ?i ?m))
			(at start (output-location ?o ?m))
			(at start (not (conveyor-full ?m)))
			(at start (not (processing ?m)))
			(at start (not (cap-buffered ?m)))
		)
		:effect (and
			(at start (processing ?m))
			(at start (conveyor-full ?m))
			(at end (not (processing ?m)))
			(at end (material-at ?o))
			(at end (cap-buffered ?m))
		)
	)

	(:durative-action mount-cap
		:parameters (?m - cap_station ?p - product ?s - step ?i - input ?o - output)
		:duration (= ?duration 25)
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
			(at end (not (processing ?m)))
			(at start (not (product-at ?p ?i)))
			(at end (product-at ?p ?o))
			(at end (not (cap-buffered ?m)))
			(at end (step-completed ?s))
		)
	)

	(:durative-action deliver
		:parameters (?p - product ?s - step)
		:duration (= ?duration 40)
		:condition (and
			(at start (product-at ?p ds_in))
			(at start (has-step ?p ?s))
			(at start (step-at-machine ?s ds))
			(at start (not (step-completed ?s)))
			(at start (not (processing ds)))
		)
		:effect (and
			(at start (processing ds))
			(at end (not (processing ds)))
			(at start (not (product-at ?p ds_in)))
			(at end (not (conveyor-full ds)))
			(at end (step-completed ?s))
			(at end (decrease (product-count) 1))
		)
	)

	(:durative-action discard-material
		:parameters ()
		:duration (= ?duration 40)
		:condition (and
			(at start (material-at ds_in))
			(at start (not (processing ds)))
		)
		:effect (and
			(at start (processing ds))
			(at end (not (processing ds)))
			(at end (not (material-at ds_in)))
			(at end (not (conveyor-full ds)))
		)
	)

	(:durative-action transport-material-discard
		:parameters (?r - robot ?o - output)
		:duration (= ?duration (path-length ?o ds_in))
		:condition (and
			(at start (robot-at ?r ?o))
			(at start (robot-holding-material ?r))
			(at end (no-robot-at-location ds_in))
			(at end (not (conveyor-full ds)))
		)
		:effect (and
			(at start (not (robot-at ?r ?o)))
			(at start (not (material-at ?o)))
			(at end (robot-at ?r ds_in))
			(at end (material-at ds_in))
			(at end (conveyor-full ds))
		)
	)

	(:durative-action transport-material-load
		:parameters (?r - robot ?o - output ?i - input ?m - ring_station)
		:duration (= ?duration (path-length ?o ?i))
		:condition (and
			(at start (input-location ?i ?m))
			(at start (robot-at ?r ?o))
			(at start (robot-holding-material ?r))
			(at end (no-robot-at-location ?i))
			(at end (< (material-load ?m) 3))
		)
		:effect (and
			(at start (not (robot-at ?r ?o)))
			(at end (robot-at ?r ?i))
			(at end (not (robot-holding-material ?r)))
			(at end (increase (material-load ?m) 1))
		)
	)

	(:durative-action transport-product
		:parameters (?r - robot ?p - product ?o - output ?i - input ?m - machine ?s1 ?s2 - step)
		:duration (= ?duration (path-length ?o ?i))
		:condition (and
			(at start (robot-holding-product ?r ?p))
			(at start (has-step ?p ?s1))
			(at start (has-step ?p ?s2))
			(at start (step-at-machine ?s2 ?m))
			(at start (step-precedes ?s1 ?s2))
			(at start (step-completed ?s1))
			(at start (input-location ?i ?m))
			(at start (not (step-completed ?s2)))
			(at start (robot-at ?r ?o))
			(at end (no-robot-at-location ?i))
			(at end (not (conveyor-full ?m)))
		)
		:effect (and
			(at start (not (robot-at ?r ?o)))
			(at end (robot-at ?r ?i))
			(at end (product-at ?p ?i))
			(at end (conveyor-full ?m))
		)
	)

	(:durative-action move-pickup-material
		:parameters (?r - robot ?i - location ?o - output ?m - machine)
		:duration (= ?duration (path-length ?i ?o))
		:condition (and
			(at start (output-location ?o ?m))
			(at start (robot-at ?r ?i))
			(at start (robot-can-pickup ?r))
			(at end (no-robot-at-location ?o))
			(at start (material-at ?o))
		)
		:effect (and
			(at start (not (robot-at ?r ?i)))
			(at end (robot-at ?r ?o))
			(at end (robot-holding-material ?r))
			(at end (not (material-at ?o)))
			(at end (not (conveyor-full ?m)))
		)
	)

	(:durative-action move-pickup-product
		:parameters (?r - robot ?i - location ?o - output ?p - product ?m - machine)
		:duration (= ?duration (path-length ?i ?o))
		:condition (and
			(at start (output-location ?o ?m))
			(at start (robot-at ?r ?i))
			(at start (robot-can-pickup ?r))
			(at end (no-robot-at-location ?o))
			(at end (product-at ?p ?o))
		)
		:effect (and
			(at start (not (robot-at ?r ?i)))
			(at end (robot-at ?r ?o))
			(at end (robot-holding-product ?r ?p))
			(at end (not (product-at ?p ?o)))
			(at end (not (conveyor-full ?m)))
		)
	)

	(:durative-action move-in
		:parameters (?r - robot)
		:duration (= ?duration 10)
		:condition (and
			(at start (robot-at-init ?r))
			(at start (robot-can-enter ?r))
			(at end (no-robot-at-location start))
		)
		:effect (and
			(at end (not (robot-at-init ?r)))
			(at end (robot-at ?r start))
		)
	)
		
	(:derived
		(robot-can-pickup ?r - robot)
		(and 
			(not (robot-holding-material ?r))
	    (not (exists (?_p - product) (robot-holding-product ?r ?_p)))
		)
	)
	
  (:derived
    (no-robot-at-location ?l - location)
    (not (exists (?_r - robot) (robot-at ?_r ?l)))
  )
  
  (:derived
    (robot-can-enter ?r - robot)
    (not (exists (?_r - robot) (and (robot-precedes ?_r ?r) (robot-at-init ?_r))))
  )

  (:derived
    (can-start-product ?p - product)
    (not (exists (?_p - product) (and (product-precedes ?_p ?p) (not (product-active ?_p)))))
  )
)

