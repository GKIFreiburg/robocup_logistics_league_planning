(define (problem rcll-production-steps_task)
(:domain rcll-production-steps)
(:objects
    R1 R2 R3 - robot
)
(:init
    (robot-at-init R1)
    (robot-at-init R2)
    (robot-at-init R3)
    (robot-precedes R1 R2)
    (robot-precedes R1 R3)
    (robot-precedes R2 R3)
    (= (material-stored C_RS1) 1)
    (= (material-stored C_RS2) 2)
    (= (path-length START C-BS-O) 15)
)
(:goal (and
    (robot-at R1 START)
)))
