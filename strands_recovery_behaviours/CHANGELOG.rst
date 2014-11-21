^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package strands_recovery_behaviours
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.0.9 (2014-11-21)
------------------

0.0.8 (2014-11-21)
------------------

0.0.7 (2014-11-21)
------------------
* updated changelogs
* magnetic strip monitor and recovery
  just asks to call a robot handler and puts a message on screen
  to restart the navigation there needss to be an explicit service call to /reset_barrier_stop done by one of us
* Contributors: Bruno Lacerda, Marc Hanheide

* magnetic strip monitor and recovery
  just asks to call a robot handler and puts a message on screen
  to restart the navigation there needss to be an explicit service call to /reset_barrier_stop done by one of us
* Contributors: Bruno Lacerda

* magnetic strip monitor and recovery
  just asks to call a robot handler and puts a message on screen
  to restart the navigation there needss to be an explicit service call to /reset_barrier_stop done by one of us
* Contributors: Bruno Lacerda

* magnetic strip monitor and recovery
  just asks to call a robot handler and puts a message on screen
  to restart the navigation there needss to be an explicit service call to /reset_barrier_stop done by one of us
* Contributors: Bruno Lacerda

* magnetic strip monitor and recovery
  just asks to call a robot handler and puts a message on screen
  to restart the navigation there needss to be an explicit service call to /reset_barrier_stop done by one of us
* Contributors: Bruno Lacerda

0.0.6 (2014-11-19)
------------------

0.0.5 (2014-11-18)
------------------
* splitting pad and service pause in config file
* Contributors: Bruno Lacerda

0.0.4 (2014-11-14)
------------------

0.0.3 (2014-11-11)
------------------

0.0.2 (2014-11-10)
------------------
* Removing monitored_navigation as required component.
  Not needed for a pseudo metapackage and breaks the build.
* Contributors: Christian Dondrup

0.0.1 (2014-11-10)
------------------
* moving mon nav config and launch files here
* renaming smach recoveries package
* fixing imports + sending number of fails when asking for help
* update dependencies
* small bug fix
* logging pauses - user taking over with gamepad or pause service calls - to mongodb
* adding mongo logging for backtrack recovery
* publishing recovery events to a topic
* waiting for backtrack action
* using param server for the recovery parameters
* update package.xml and CMakeLists.txt
* re-adding possibility of pausing monitored navigation via gamepad or service call
* moving human_help_manager service definition to human_help_manager package
* removing monitored nav readme
* removing monitored_navigation filesthat shoundlnt be here
* implementation of recovery state machines for monitored navigation
* Contributors: Bruno Lacerda
