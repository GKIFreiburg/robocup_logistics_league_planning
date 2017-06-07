(define (problem rcll-production-steps_task)
(:domain rcll-production-steps)
(:objects
    p10 - product
    r1 r2 r3 - robot
    red_base_p10 black_cap_p10 gate1_delivery_p10 - step
)
(:init
    (cap-buffered cs2)
    (has-step p10 red_base_p10)
    (has-step p10 black_cap_p10)
    (has-step p10 gate1_delivery_p10)
    (initial-step red_base_p10)
    (input-location cs1_in cs1)
    (input-location cs2_in cs2)
    (input-location rs1_in rs1)
    (input-location rs2_in rs2)
    (input-location ds_in ds)
    (location-occupied )
    (location-occupied cs2_in)
    (material-at cs2_out)
    (output-full bs_out)
    (output-full cs2_out)
    (output-location cs1_out cs1)
    (output-location cs2_out cs2)
    (output-location rs1_out rs1)
    (output-location rs2_out rs2)
    (output-location bs_out bs)
    (product-at p10 bs_out)
    (robot-at r1 )
    (robot-at r1 cs2_in)
    (robot-precedes r1 r2)
    (robot-precedes r1 r3)
    (robot-precedes r2 r3)
    (step-at-machine red_base_p10 bs)
    (step-at-machine black_cap_p10 cs2)
    (step-at-machine gate1_delivery_p10 ds)
    (step-completed red_base_p10)
    (step-precedes red_base_p10 black_cap_p10)
    (step-precedes black_cap_p10 gate1_delivery_p10)
    (= (path-length bs_out cs1_in) 61.3515)
    (= (path-length bs_out cs1_out) 69.1673)
    (= (path-length bs_out cs2_in) 55.6966)
    (= (path-length bs_out cs2_out) 44.362)
    (= (path-length bs_out ds_in) 44.3647)
    (= (path-length bs_out rs1_in) 24.2634)
    (= (path-length bs_out rs1_out) 34.7227)
    (= (path-length bs_out rs2_in) 28.2018)
    (= (path-length bs_out rs2_out) 15.4633)
    (= (path-length cs1_in bs_out) 61.3515)
    (= (path-length cs1_in cs1_out) 12.6618)
    (= (path-length cs1_in cs2_in) 22.0476)
    (= (path-length cs1_in cs2_out) 21.3711)
    (= (path-length cs1_in ds_in) 26.7664)
    (= (path-length cs1_in rs1_in) 42.2187)
    (= (path-length cs1_in rs1_out) 31.4303)
    (= (path-length cs1_in rs2_in) 47.1389)
    (= (path-length cs1_in rs2_out) 58.7964)
    (= (path-length cs1_out bs_out) 69.1673)
    (= (path-length cs1_out cs1_in) 12.6618)
    (= (path-length cs1_out cs2_in) 21.2787)
    (= (path-length cs1_out cs2_out) 29.1869)
    (= (path-length cs1_out ds_in) 36.1518)
    (= (path-length cs1_out rs1_in) 49.7733)
    (= (path-length cs1_out rs1_out) 39.2462)
    (= (path-length cs1_out rs2_in) 54.9547)
    (= (path-length cs1_out rs2_out) 66.6123)
    (= (path-length cs2_in bs_out) 55.6966)
    (= (path-length cs2_in cs1_in) 22.0476)
    (= (path-length cs2_in cs1_out) 21.2787)
    (= (path-length cs2_in cs2_out) 24.3618)
    (= (path-length cs2_in ds_in) 33.9987)
    (= (path-length cs2_in rs1_in) 35.7368)
    (= (path-length cs2_in rs1_out) 35.6468)
    (= (path-length cs2_in rs2_in) 51.3554)
    (= (path-length cs2_in rs2_out) 59.5605)
    (= (path-length cs2_out bs_out) 44.362)
    (= (path-length cs2_out cs1_in) 21.3711)
    (= (path-length cs2_out cs1_out) 29.1869)
    (= (path-length cs2_out cs2_in) 24.3618)
    (= (path-length cs2_out ds_in) 25.113)
    (= (path-length cs2_out rs1_in) 24.4023)
    (= (path-length cs2_out rs1_out) 26.3559)
    (= (path-length cs2_out rs2_in) 42.0644)
    (= (path-length cs2_out rs2_out) 48.2259)
    (= (path-length ds_in bs_out) 44.3647)
    (= (path-length ds_in cs1_in) 26.7664)
    (= (path-length ds_in cs1_out) 36.1518)
    (= (path-length ds_in cs2_in) 33.9987)
    (= (path-length ds_in cs2_out) 25.113)
    (= (path-length ds_in rs1_in) 25.2439)
    (= (path-length ds_in rs1_out) 14.4435)
    (= (path-length ds_in rs2_in) 28.8725)
    (= (path-length ds_in rs2_out) 40.5301)
    (= (path-length rs1_in bs_out) 24.2634)
    (= (path-length rs1_in cs1_in) 42.2187)
    (= (path-length rs1_in cs1_out) 49.7733)
    (= (path-length rs1_in cs2_in) 35.7368)
    (= (path-length rs1_in cs2_out) 24.4023)
    (= (path-length rs1_in ds_in) 25.2439)
    (= (path-length rs1_in rs1_out) 25.8094)
    (= (path-length rs1_in rs2_in) 32.1371)
    (= (path-length rs1_in rs2_out) 28.1273)
    (= (path-length rs1_out bs_out) 34.7227)
    (= (path-length rs1_out cs1_in) 31.4303)
    (= (path-length rs1_out cs1_out) 39.2462)
    (= (path-length rs1_out cs2_in) 35.6468)
    (= (path-length rs1_out cs2_out) 26.3559)
    (= (path-length rs1_out ds_in) 14.4435)
    (= (path-length rs1_out rs1_in) 25.8094)
    (= (path-length rs1_out rs2_in) 20.5101)
    (= (path-length rs1_out rs2_out) 32.1676)
    (= (path-length rs2_in bs_out) 28.2018)
    (= (path-length rs2_in cs1_in) 47.1389)
    (= (path-length rs2_in cs1_out) 54.9547)
    (= (path-length rs2_in cs2_in) 51.3554)
    (= (path-length rs2_in cs2_out) 42.0644)
    (= (path-length rs2_in ds_in) 28.8725)
    (= (path-length rs2_in rs1_in) 32.1371)
    (= (path-length rs2_in rs1_out) 20.5101)
    (= (path-length rs2_in rs2_out) 14.1437)
    (= (path-length rs2_out bs_out) 15.4633)
    (= (path-length rs2_out cs1_in) 58.7964)
    (= (path-length rs2_out cs1_out) 66.6123)
    (= (path-length rs2_out cs2_in) 59.5605)
    (= (path-length rs2_out cs2_out) 48.2259)
    (= (path-length rs2_out ds_in) 40.5301)
    (= (path-length rs2_out rs1_in) 28.1273)
    (= (path-length rs2_out rs1_out) 32.1676)
    (= (path-length rs2_out rs2_in) 14.1437)
    (= (path-length start bs_out) 46.1818)
    (= (path-length start cs1_in) 32.6634)
    (= (path-length start cs1_out) 28.236)
    (= (path-length start cs2_in) 14.1995)
    (= (path-length start cs2_out) 14.8469)
    (= (path-length start ds_in) 36.4053)
    (= (path-length start rs1_in) 26.222)
    (= (path-length start rs1_out) 37.6482)
    (= (path-length start rs2_in) 53.3567)
    (= (path-length start rs2_out) 50.0457)
)
(:goal (and
    (step-completed red_base_p10)
    (step-completed black_cap_p10)
    (step-completed gate1_delivery_p10)
)))
