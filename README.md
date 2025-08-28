# Mixed-criticality-scheduling-on-FreeRTOS

My project aims to extend the scheduler and the task data structures of FreeRTOS for supporting an alternative scheduling algorithm, as it currently does not comply with the standard notation for real-time tasks analysis. By default, FreeRTOS uses a fixed-priority preemptive scheduling policy, with round-robin time-slicing of equal priority tasks: in this project it has been extended to support the Early Deadline First with Virtual Deadline scheduling policy for systems with 2 criticality levels, EDF-VD(2).

In order to extend FreeRTOS to support the scheduling algorithm EDF-VD(2), I had to work mainly on the Hardware Independent Code, in particular on tasks.c. In this file we find the entire life cycle of the task, from its creation to its suspension, and all the functions related to scheduling, from content switch to list management.

The algorithm is suited to work in an environment where these configurations applies (in FreeRTOSConfig.h): preemption disabled, static allocation disabled, dynamic allocation enabled and tick-hook enabled.

The final product of my project is a version of FreeRTOS that, based on how it is initially configured, has a scheduler that manages tasks with different scheduling policies: a fixed-priority preemptive scheduling policy, with round-robin time-slicing of equal priority tasks (native scheduling algorithm), Early Deadline First non-preemptive that works in both an implicit-deadline and constrained-deadline system and, what is the final goal of the project, Early Deadline First with Virtual Deadline for systems with 2 criticality levels, again in both an implicit-deadline and constrained-deadline system, non-preemptive.
