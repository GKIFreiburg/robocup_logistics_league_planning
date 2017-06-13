(define (problem rcll-production-steps_task)
(:domain rcll-production-steps)
(:objects
    p10 - product
    r1 r2 r3 - robot
<<<<<<< HEAD
    silver_base_p20 black_cap_p20 gate1_delivery_p20 - step
)
(:init
    (cap-buffered cs1)
    (cap-buffered cs2)
    (conveyor-full cs1)
    (has-step p20 silver_base_p20)
    (has-step p20 black_cap_p20)
    (has-step p20 gate1_delivery_p20)
    (initial-step silver_base_p20)
=======
    black_base_p10 grey_cap_p10 gate3_delivery_p10 - step
)
(:init
    (has-step p10 black_base_p10)
    (has-step p10 grey_cap_p10)
    (has-step p10 gate3_delivery_p10)
    (initial-step black_base_p10)
>>>>>>> b8c99c076fa863bde049d08363c216e4d97b9e5f
    (input-location cs1_in cs1)
    (input-location cs2_in cs2)
    (input-location rs1_in rs1)
    (input-location rs2_in rs2)
    (input-location ds_in ds)
<<<<<<< HEAD
    (location-occupied rs1_in)
    (location-occupied cs2_in)
    (location-occupied ds_in)
    (material-at cs1_out)
    (material-at cs2_out)
=======
>>>>>>> b8c99c076fa863bde049d08363c216e4d97b9e5f
    (output-location cs1_out cs1)
    (output-location cs2_out cs2)
    (output-location rs1_out rs1)
    (output-location rs2_out rs2)
    (output-location bs_out bs)
<<<<<<< HEAD
    (output-location bs_in bs)
    (robot-at r2 rs1_in)
    (robot-at r3 cs2_in)
    (robot-at r1 ds_in)
    (robot-precedes r1 r2)
    (robot-precedes r1 r3)
    (robot-precedes r2 r3)
    (step-at-machine silver_base_p20 bs)
    (step-at-machine black_cap_p20 cs2)
    (step-at-machine gate1_delivery_p20 ds)
    (step-precedes silver_base_p20 black_cap_p20)
    (step-precedes black_cap_p20 gate1_delivery_p20)
    (= (material-stored rs1) 1)
    (= (material-stored rs2) 0)
    (= (path-length bs_in bs_out) 9.80499)
    (= (path-length bs_in cs1_in) 56.2377)
    (= (path-length bs_in cs1_out) 63.8106)
    (= (path-length bs_in cs2_in) 50.0478)
    (= (path-length bs_in cs2_out) 39.3321)
    (= (path-length bs_in ds_in) 39.1852)
    (= (path-length bs_in rs1_in) 19.127)
    (= (path-length bs_in rs1_out) 30.0413)
    (= (path-length bs_in rs2_in) 21.2672)
    (= (path-length bs_in rs2_out) 11.5575)
    (= (path-length bs_out bs_in) 9.80499)
    (= (path-length bs_out cs1_in) 61.5629)
    (= (path-length bs_out cs1_out) 69.1358)
    (= (path-length bs_out cs2_in) 55.373)
    (= (path-length bs_out cs2_out) 44.6572)
    (= (path-length bs_out ds_in) 44.5104)
    (= (path-length bs_out rs1_in) 24.4522)
    (= (path-length bs_out rs1_out) 35.3664)
    (= (path-length bs_out rs2_in) 28.662)
    (= (path-length bs_out rs2_out) 15.5836)
    (= (path-length cs1_in bs_in) 56.2377)
    (= (path-length cs1_in bs_out) 61.5629)
    (= (path-length cs1_in cs1_out) 12.9856)
    (= (path-length cs1_in cs2_in) 21.4872)
    (= (path-length cs1_in cs2_out) 20.915)
    (= (path-length cs1_in ds_in) 26.9309)
    (= (path-length cs1_in rs1_in) 41.8211)
    (= (path-length cs1_in rs1_out) 31.225)
    (= (path-length cs1_in rs2_in) 47.0858)
    (= (path-length cs1_in rs2_out) 58.4958)
    (= (path-length cs1_out bs_in) 63.8106)
    (= (path-length cs1_out bs_out) 69.1358)
    (= (path-length cs1_out cs1_in) 12.9856)
    (= (path-length cs1_out cs2_in) 21.0577)
    (= (path-length cs1_out cs2_out) 29.0468)
    (= (path-length cs1_out ds_in) 36.2971)
    (= (path-length cs1_out rs1_in) 48.9831)
    (= (path-length cs1_out rs1_out) 39.3569)
    (= (path-length cs1_out rs2_in) 55.2177)
    (= (path-length cs1_out rs2_out) 66.6277)
    (= (path-length cs2_in bs_in) 50.0478)
    (= (path-length cs2_in bs_out) 55.373)
    (= (path-length cs2_in cs1_in) 21.4872)
    (= (path-length cs2_in cs1_out) 21.0577)
    (= (path-length cs2_in cs2_out) 23.8494)
    (= (path-length cs2_in ds_in) 32.989)
    (= (path-length cs2_in rs1_in) 35.2204)
    (= (path-length cs2_in rs1_out) 35.3077)
    (= (path-length cs2_in rs2_in) 51.1685)
    (= (path-length cs2_in rs2_out) 59.6831)
    (= (path-length cs2_out bs_in) 39.3321)
    (= (path-length cs2_out bs_out) 44.6572)
    (= (path-length cs2_out cs1_in) 20.915)
    (= (path-length cs2_out cs1_out) 29.0468)
    (= (path-length cs2_out cs2_in) 23.8494)
    (= (path-length cs2_out ds_in) 25.2384)
    (= (path-length cs2_out rs1_in) 24.5046)
    (= (path-length cs2_out rs1_out) 25.8839)
    (= (path-length cs2_out rs2_in) 41.7447)
    (= (path-length cs2_out rs2_out) 48.9673)
    (= (path-length ds_in bs_in) 39.1852)
    (= (path-length ds_in bs_out) 44.5104)
    (= (path-length ds_in cs1_in) 26.9309)
    (= (path-length ds_in cs1_out) 36.2971)
    (= (path-length ds_in cs2_in) 32.989)
    (= (path-length ds_in cs2_out) 25.2384)
    (= (path-length ds_in rs1_in) 25.4461)
    (= (path-length ds_in rs1_out) 14.1725)
    (= (path-length ds_in rs2_in) 29.2344)
    (= (path-length ds_in rs2_out) 40.6444)
    (= (path-length rs1_in bs_in) 19.127)
    (= (path-length rs1_in bs_out) 24.4522)
    (= (path-length rs1_in cs1_in) 41.8211)
    (= (path-length rs1_in cs1_out) 48.9831)
    (= (path-length rs1_in cs2_in) 35.2204)
    (= (path-length rs1_in cs2_out) 24.5046)
    (= (path-length rs1_in ds_in) 25.4461)
    (= (path-length rs1_in rs1_out) 26.0916)
    (= (path-length rs1_in rs2_in) 32.1897)
    (= (path-length rs1_in rs2_out) 28.7623)
    (= (path-length rs1_out bs_in) 30.0413)
    (= (path-length rs1_out bs_out) 35.3664)
    (= (path-length rs1_out cs1_in) 31.225)
    (= (path-length rs1_out cs1_out) 39.3569)
    (= (path-length rs1_out cs2_in) 35.3077)
    (= (path-length rs1_out cs2_out) 25.8839)
    (= (path-length rs1_out ds_in) 14.1725)
    (= (path-length rs1_out rs1_in) 26.0916)
    (= (path-length rs1_out rs2_in) 20.8894)
    (= (path-length rs1_out rs2_out) 32.2994)
    (= (path-length rs2_in bs_in) 21.2672)
    (= (path-length rs2_in bs_out) 28.662)
    (= (path-length rs2_in cs1_in) 47.0858)
    (= (path-length rs2_in cs1_out) 55.2177)
    (= (path-length rs2_in cs2_in) 51.1685)
    (= (path-length rs2_in cs2_out) 41.7447)
    (= (path-length rs2_in ds_in) 29.2344)
    (= (path-length rs2_in rs1_in) 32.1897)
    (= (path-length rs2_in rs1_out) 20.8894)
    (= (path-length rs2_in rs2_out) 14.2613)
    (= (path-length rs2_out bs_in) 11.5575)
    (= (path-length rs2_out bs_out) 15.5836)
    (= (path-length rs2_out cs1_in) 58.4958)
    (= (path-length rs2_out cs1_out) 66.6277)
    (= (path-length rs2_out cs2_in) 59.6831)
    (= (path-length rs2_out cs2_out) 48.9673)
    (= (path-length rs2_out ds_in) 40.6444)
    (= (path-length rs2_out rs1_in) 28.7623)
    (= (path-length rs2_out rs1_out) 32.2994)
    (= (path-length rs2_out rs2_in) 14.2613)
    (= (path-length start bs_in) 7.3014)
    (= (path-length start bs_out) 12.2342)
    (= (path-length start cs1_in) 53.621)
    (= (path-length start cs1_out) 61.1939)
    (= (path-length start cs2_in) 47.4312)
    (= (path-length start cs2_out) 36.7154)
    (= (path-length start ds_in) 36.5685)
    (= (path-length start rs1_in) 16.5103)
    (= (path-length start rs1_out) 27.4246)
    (= (path-length start rs2_in) 26.6464)
    (= (path-length start rs2_out) 16.9367)
)
(:goal (and
    (cap-buffered cs1)
    (cap-buffered cs2)
    (step-completed silver_base_p20)
    (step-completed black_cap_p20)
    (step-completed gate1_delivery_p20)
=======
    (robot-at-init r1)
    (robot-precedes r1 r2)
    (robot-precedes r1 r3)
    (robot-precedes r2 r3)
    (step-at-machine black_base_p10 bs)
    (step-at-machine grey_cap_p10 cs1)
    (step-at-machine gate3_delivery_p10 ds)
    (step-precedes black_base_p10 grey_cap_p10)
    (step-precedes grey_cap_p10 gate3_delivery_p10)
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
    (step-completed black_base_p10)
    (step-completed grey_cap_p10)
    (step-completed gate3_delivery_p10)
>>>>>>> b8c99c076fa863bde049d08363c216e4d97b9e5f
)))
