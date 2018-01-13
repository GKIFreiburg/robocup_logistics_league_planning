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

	(:durative-action prepare-dispense-material
		:parameters (?m - base_station ?o - output)
		:duration (= ?duration 0.01)
		:condition (and
			(at start (conveyor-empty ?m))
			(at start (station-idle ?m))
      (at start (output-location ?o ?m))
		)
		:effect (and
			(at start (not (station-idle ?m)))
			(at end (station-prepared-for-dispense ?m))
			(at end (prepared-dispense-output ?o))
		)
	)

	(:durative-action dispense-material
		:parameters (?m - base_station ?o - output)
		:duration (= ?duration 1)
		:condition (and
			(at start (conveyor-empty ?m))
			(at start (station-prepared-for-dispense ?m))
			(at start (prepared-dispense-output ?o))
		)
		:effect (and
			(at start (not (station-prepared-for-dispense ?m)))
			(at start (not (prepared-dispense-output ?o)))
			(at start (not (conveyor-empty ?m)))
			(at end (station-output-ready ?m))
			(at end (material-at ?o))
		)
	)

	(:durative-action prepare-mount-cap
		:parameters (?r - robot ?p - product ?s2 ?s - step ?m - cap_station ?i - input)
		:duration (= ?duration 0.01)
		:condition (and
			(at start (conveyor-empty ?m))
			(at start (station-idle ?m))
      (at start (input-location ?i ?m))
      (at start (robot-at ?r ?i))
      (at start (step-incomplete ?s))
      (at start (step-precedes ?s2 ?s))
      (at start (step-completed ?s2))
      (at start (has-step ?p ?s2))
      (at start (has-step ?p ?s))
			(at start (step-at-station ?s ?m))
      (at start (robot-holding-product ?r ?p))
			(at start (cap-buffered ?m))
		)
		:effect (and
			(at start (not (station-idle ?m)))
			(at end (station-prepared-for-step ?m ?s))
		)
	)

	(:durative-action prepare-mount-ring
		:parameters (?r - robot ?p - product ?s2 ?s - step ?m - ring_station ?i - input ?c1 ?c2 - material_counter)
		:duration (= ?duration 0.01)
		:condition (and
			(at start (conveyor-empty ?m))
			(at start (station-idle ?m))
      (at start (input-location ?i ?m))
      (at start (robot-at ?r ?i))
      (at start (step-incomplete ?s))
      (at start (step-precedes ?s2 ?s))
      (at start (step-completed ?s2))
      (at start (has-step ?p ?s2))
      (at start (has-step ?p ?s))
			(at start (step-at-station ?s ?m))
      (at start (robot-holding-product ?r ?p))
			(at start (material-stored ?m ?c2))
			(at start (material-required ?s ?c1))
			(at start (less-or-equal ?c1 ?c2))
		)
		:effect (and
			(at start (not (station-idle ?m)))
			(at end (station-prepared-for-step ?m ?s))
		)
	)

	(:durative-action prepare-dispense-product
		:parameters (?s - step ?m - base_station ?o - output)
		:duration (= ?duration 0.01)
		:condition (and
			(at start (conveyor-empty ?m))
			(at start (station-idle ?m))
      (at start (output-location ?o ?m))
      (at start (step-incomplete ?s))
      (at start (initial-step ?s))
			(at start (step-at-station ?s ?m))
		)
		:effect (and
			(at start (not (station-idle ?m)))
			(at end (station-prepared-for-step ?m ?s))
		)
	)

	(:durative-action prepare-discard
		:parameters (?r - robot ?m - delivery_station ?i - input)
		:duration (= ?duration 0.01)
		:condition (and
			(at start (conveyor-empty ?m))
			(at start (station-idle ?m))
      (at start (input-location ?i ?m))
      (at start (robot-at ?r ?i))
      (at start (robot-holding-material ?r))
		)
		:effect (and
			(at start (not (station-idle ?m)))
			(at end (station-prepared-for-discard ?m))
		)
	)

	(:durative-action prepare-delivery
		:parameters (?r - robot ?p - product ?s2 ?s - step ?m - delivery_station ?i - input)
		:duration (= ?duration 0.01)
		:condition (and
			(at start (conveyor-empty ?m))
			(at start (station-idle ?m))
      (at start (input-location ?i ?m))
      (at start (robot-at ?r ?i))
      (at start (robot-holding-product ?r ?p))
      (at start (step-incomplete ?s))
      (at start (step-precedes ?s2 ?s))
      (at start (step-completed ?s2))
      (at start (has-step ?p ?s2))
      (at start (has-step ?p ?s))
			(at start (step-at-station ?s ?m))
		)
		:effect (and
			(at start (not (station-idle ?m)))
			(at end (station-prepared-for-step ?m ?s))
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
			(at start (station-prepared-for-step ?m ?s))
      (at start (output-location ?o ?m))
		)
		:effect (and
			(at start (not (station-prepared-for-step ?m ?s)))
			(at start (not (conveyor-empty ?m)))
			(at end (station-output-ready ?m))
			(at end (product-at ?p ?o))
			(at end (not (step-incomplete ?s)))
			(at end (step-completed ?s))
		)
	)

	(:durative-action mount-ring
		:parameters (?m - ring_station ?p - product ?s - step ?i - input ?o - output ?mi ?mr ?mf - material_counter)
		:duration (= ?duration 1)
		:condition (and
			(at start (product-at ?p ?i))
			(at start (has-step ?p ?s))
			(at start (step-at-station ?s ?m))
			(at start (step-incomplete ?s))
			(at start (input-location ?i ?m))
			(at start (output-location ?o ?m))
			(at start (station-prepared-for-step ?m ?s))
			(at start (material-required ?s ?mr))
			(at start (material-stored ?m ?mi))
			(at start (subtract ?mi ?mr ?mf))
		)
		:effect (and
			(at start (not (station-prepared-for-step ?m ?s)))
			(at start (not (product-at ?p ?i)))
			(at start (not (material-stored ?m ?mi)))
			(at end (material-stored ?m ?mf))
			(at end (station-output-ready ?m))
			(at end (product-at ?p ?o))
			(at end (not (step-incomplete ?s)))
			(at end (step-completed ?s))
		)
	)

	(:durative-action prepare-buffer-cap
		:parameters (?r - robot ?m - cap_station ?i - input)
		:duration (= ?duration 0.01)
		:condition (and
			(at start (station-idle ?m))
			(at start (cap-buffer-empty ?m))
			(at start (input-location ?i ?m))
			(at start (robot-at ?r ?i))
		)
		:effect (and
			(at start (not (station-idle ?m)))
			(at end (station-prepared-for-cap ?m))
		)
	)

	(:durative-action buffer-cap
		:parameters (?m - cap_station ?i - input ?o - output)
		:duration (= ?duration 1)
		:condition (and
			(at start (input-location ?i ?m))
			(at start (output-location ?o ?m))
			(at start (material-at ?i))
			(at start (station-prepared-for-cap ?m))
			(at start (cap-buffer-empty ?m))
		)
		:effect (and
			(at start (not (station-prepared-for-cap ?m)))
			(at start (not (material-at ?i)))
			(at end (station-output-ready ?m))
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
			(at start (station-prepared-for-step ?m ?s))
			(at start (cap-buffered ?m))
		)
		:effect (and
			(at start (not (station-prepared-for-step ?m ?s)))
			(at start (not (product-at ?p ?i)))
			(at end (station-output-ready ?m))
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
			(at start (station-prepared-for-step ?m ?s))
		)
		:effect (and
			(at start (not (station-prepared-for-step ?m ?s)))
			(at start (not (product-at ?p ?i)))
			(at end (station-idle ?m))
			(at end (conveyor-empty ?m))
			(at end (step-completed ?s))
			(at end (not (step-incomplete ?s)))
		)
	)

	(:durative-action discard-material
		:parameters (?m - delivery_station ?i - input)
		:duration (= ?duration 1)
		:condition (and
			(at start (material-at ?i))
			(at start (input-location ?i ?m))
			(at start (station-prepared-for-discard ?m))
		)
		:effect (and
			(at start (not (station-prepared-for-discard ?m)))
			(at end (station-idle ?m))
			(at end (not (material-at ?i)))
			(at end (conveyor-empty ?m))
		)
	)

	(:durative-action insert-cap
		:parameters (?r - robot ?m - cap_station ?i - input)
		:duration (= ?duration 30)
		:condition (and
			(over all (station-prepared-for-cap ?m))
			(over all (robot-at ?r ?i))
			(at start (robot-idle ?r))
			(at start (robot-gripper-free ?r))
			(at start (input-location ?i ?m))
			(at start (conveyor-empty ?m))
		)
		:effect (and
			(at start (not (conveyor-empty ?m)))
			(at start (not (robot-idle ?r)))
			(at end (robot-idle ?r))
			(at end (material-at ?i))
		)
	)

	(:durative-action pickup-material
		:parameters (?r - robot ?o - output ?om - station)
		:duration (= ?duration 15)
		:condition (and
			(at start (robot-idle ?r))
			(at start (output-location ?o ?om))
			(at start (robot-at ?r ?o))
			(at start (material-at ?o))
			(at start (robot-gripper-free ?r))
			(at start (station-output-ready ?om))
		)
		:effect (and
			(at start (not (robot-idle ?r)))
			(at start (not (material-at ?o)))
			(at end (not (station-output-ready ?om)))
			(at end (station-idle ?om))
			(at end (conveyor-empty ?om))
			(at end (robot-holding-material ?r))
			(at end (not (robot-gripper-free ?r)))
			(at end (robot-idle ?r))
		)
	)

	(:durative-action insert-material
		:parameters (?r - robot ?i - input ?im - delivery_station)
		:duration (= ?duration 15)
		:condition (and
			(at start (robot-idle ?r))
			(at start (input-location ?i ?im))
			(at start (robot-at ?r ?i))
			(at start (robot-holding-material ?r))
			(at start (station-prepared-for-discard ?im))
		)
		:effect (and
			(at start (not (robot-idle ?r)))
			(at start (not (conveyor-empty ?im)))
			(at end (material-at ?i))
			(at end (not (robot-holding-material ?r)))
			(at end (robot-gripper-free ?r))
			(at end (robot-idle ?r))
		)
	)

	(:durative-action insert-material-slide
		:parameters (?r - robot ?i - input ?im - ring_station ?mi ?mf - material_counter)
		:duration (= ?duration 15)
		:condition (and
			(at start (robot-idle ?r))
			(at start (input-location ?i ?im))
			(at start (robot-at ?r ?i))
			(at start (robot-holding-material ?r))
			(over all (material-stored ?im ?mi))
			(over all (add-one ?mi ?mf))
		)
		:effect (and
			(at start (not (robot-idle ?r)))
			(at end (not (robot-holding-material ?r)))
			(at end (robot-gripper-free ?r))
			(at end (robot-idle ?r))
			(at end (not (material-stored ?im ?mi)))
			(at end (material-stored ?im ?mf))
		)
	)

	(:durative-action transport-material
		:parameters (?r - robot ?o - output ?i - input ?m - station)
		:duration (= ?duration (path-length ?o ?i))
		:condition (and
			(at start (robot-idle ?r))
			(at start (input-location ?i ?m))
			(at start (robot-at ?r ?o))
			(at start (robot-holding-material ?r))
		)
		:effect (and
			(at start (not (robot-idle ?r)))
			(at start (not (robot-at ?r ?o)))
			(at end (robot-idle ?r))
			(at end (robot-at ?r ?i))
		)
	)

	(:durative-action pickup-product
		:parameters (?r - robot ?p - product ?o - output ?om - station)
		:duration (= ?duration 15)
		:condition (and
			(at start (robot-idle ?r))
			(at start (output-location ?o ?om))
			(at start (robot-at ?r ?o))
			(at start (product-at ?p ?o))
			(at start (robot-gripper-free ?r))
			(at start (station-output-ready ?om))
		)
		:effect (and
			(at start (not (robot-idle ?r)))
			(at start (not (station-output-ready ?om)))
			(at end (station-idle ?om))
			(at end (not (product-at ?p ?o)))
			(at end (conveyor-empty ?om))
			(at end (robot-holding-product ?r ?p))
			(at end (not (robot-gripper-free ?r)))
			(at end (robot-idle ?r))
		)
	)

	(:durative-action insert-product
		:parameters (?r - robot ?p - product ?s - step ?i - input ?im - station)
		:duration (= ?duration 15)
		:condition (and
			(at start (robot-idle ?r))
			(at start (input-location ?i ?im))
			(at start (robot-at ?r ?i))
			(at start (robot-holding-product ?r ?p))
			(at start (station-prepared-for-step ?im ?s))
			(at start (has-step ?p ?s))
		)
		:effect (and
			(at start (not (robot-idle ?r)))
			(at start (not (conveyor-empty ?im)))
			(at end (product-at ?p ?i))
			(at end (not (robot-holding-product ?r ?p)))
			(at end (robot-gripper-free ?r))
			(at end (robot-idle ?r))
		)
	)

	(:durative-action transport-product
		:parameters (?r - robot ?p - product ?o - output ?i - input ?m - station ?s1 ?s2 - step)
		:duration (= ?duration (path-length ?o ?i))
		:condition (and
			(at start (robot-idle ?r))
			(at start (robot-holding-product ?r ?p))
			(at start (has-step ?p ?s1))
			(at start (has-step ?p ?s2))
			(at start (step-at-station ?s2 ?m))
			(at start (step-precedes ?s1 ?s2))
			(at start (step-completed ?s1))
			(at start (step-incomplete ?s2))
			(at start (input-location ?i ?m))
			(at start (robot-at ?r ?o))
		)
		:effect (and
			(at start (not (robot-at ?r ?o)))
			(at start (not (robot-idle ?r)))
			(at end (robot-idle ?r))
			(at end (robot-at ?r ?i))
		)
	)

	(:durative-action move
		:parameters (?r - robot ?l1 ?l2 - location)
		:duration (= ?duration (path-length ?l1 ?l2))
		:condition (and
			(at start (robot-idle ?r))
			(at start (robot-at ?r ?l1))
			(at start (robot-gripper-free ?r))
		)
		:effect (and
			(at start (not (robot-at ?r ?l1)))
			(at start (not (robot-idle ?r)))
			(at end (robot-idle ?r))
			(at end (robot-at ?r ?l2))
		)
	)

	(:durative-action move-in
		:parameters (?r - robot ?l - s_location)
		:duration (= ?duration 10)
		:condition (and
			(at start (robot-idle ?r))
			(at start (robot-outside ?r))
		)
		:effect (and
			(at start (not (robot-idle ?r)))
			(at end (not (robot-outside ?r)))
			(at end (robot-idle ?r))
			(at end (robot-at ?r ?l))
		)
	)

	(:durative-action move-out
		:parameters (?r - robot ?l - s_location)
		:duration (= ?duration 10)
		:condition (and
			(at start (robot-idle ?r))
			(at start (robot-at ?r ?l))
		)
		:effect (and
			(at start (not (robot-idle ?r)))
			(at start (not (robot-at ?r ?l)))
			(at end (robot-idle ?r))
			(at end (robot-outside ?r))
		)
	)
)

