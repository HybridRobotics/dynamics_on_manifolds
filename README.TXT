Highly Dynamic Robotics Lab
Symbolic Algebra Package for Dynamical Modeling


The package symbolic_algebra will compute the dynamical model for a user specified mechanical system. This file will cover everything from setting up your environment to configuring parameters specific to your mechanical system.

Below are the steps for downloading scala and setting up your computational environment.

1. Download the latest version of scala at http://www.scala-lang.org/download/

2. Install Scala.

3. Open command prompt (windows) or terminal (mac). Type scala while in the DynamicalModeling directory.

    If the scala interpreter does not initialize, you must set environmental variables properly.
    http://www.scala-lang.org/download/install.html

    Otherwise you can access the interpreter via $INSTALLATION_PATH$/bin/scala.

4. Build symbolic algebra package and examples from dynamics_on_manifolds directory. 
   (“$>" marks interpreter prompt)

    $> scalac mechanical_systems/*.scala src/symbolic_algebra/*/*.scala

5. Type into scala interpreter:

    $> import symbolic_algebra._
    $> SphericalPendulum.main()

    The interpreter should ouput the dynamical model.

You have now successfully set up your computational environment and are ready for modeling.



Located in the dynamics_on_manifolds package are a set of starter files, which involve configuration setups for a variety of mechanical systems. They include:

    SphericalPendulum.scala
    DoubleSphericalPendulum.scala
    Pendulum3D.scala
    DoublePendulum3D.scala
    ThreeLinkWalker.scala
    ThreeLinkWalkerFlight.scala
    QuadrotorWithLoad.scala
    QuadrotorWithContinuum.scala

The symbolic evaluator requires the system's lagrangian, infinitesimal work, and configuration variables. These are described in detail, particularly in SphericalPendulum.scala. Walk through the set up for this file for guidance on how to model systems which act on S^2. Pendulum3D will provide useful insight on how to model a system which acts on SO(3).