# Symbolic Algebra Package for Computing Symbolic Dynamics on Nonlinear Manifolds
The package `dynamics_on_manifolds` will compute the dynamical model for a user specified mechanical system. Dynamics are computed on the smooth manifolds `R^3, S^2, SO(3)` and their product spaces, for instance quadrotor with suspended load on `R^3 x S^2 x SO3`. The package also variation linearizes the dynamics to generate [Variation based linearized dynamics](https://hybrid-robotics.berkeley.edu/publications/Access2015_VariationLinearization.pdf). Finally, the package outputs the dynamics as a `latex` file and generates a `Matlab` function for the same. 


## Citation

- (TODO: ArXiv paper link)


- Bittner, Brian, and Koushil Sreenath. "Symbolic computation of dynamics on smooth manifolds." Algorithmic Foundations of Robotics XII. Springer, Cham, 2020. 336-351.
    ```
    @incollection{bittner2020symbolic,
      title={Symbolic computation of dynamics on smooth manifolds},
      author={Bittner, Brian and Sreenath, Koushil},
      booktitle={Algorithmic Foundations of Robotics XII},
      pages={336--351},
      year={2020},
      publisher={Springer}
    }
    ```

## Documentation

The documentation is available at [https://github.com/HybridRobotics/dynamics_on_manifolds/wiki](https://github.com/HybridRobotics/dynamics_on_manifolds/wiki). 

## How to Use
Please find the detailed documentation on Scala here: [https://docs.scala-lang.org/](https://docs.scala-lang.org/)

Dynamics on Manifolds packages makes use of Scala and sbt (Scala/Simple-Build-Tool). Working with the IntelliJ IDE would be useful in debugging and developing. Find more about IDE [here](https://docs.scala-lang.org/getting-started/intellij-track/getting-started-with-scala-in-intellij.html).    

### Scala Installation

#### Windows

-  TODO

#### Ubuntu
(The dynamics on manifold package is tested on Ubuntu 18.04)
1. Install sbt in terminal as follows, [source](https://www.scala-sbt.org/1.x/docs/Installing-sbt-on-Linux.html).
    ```
    echo "deb https://dl.bintray.com/sbt/debian /" | sudo tee -a /etc/apt/sources.list.d/sbt.list
    curl -sL "https://keyserver.ubuntu.com/pks/lookup?op=get&search=0x2EE0EA64E40A89B84B2DF73499E82A75642AC823" | sudo apt-key add
    sudo apt-get update
    sudo apt-get install sbt
    ``` 
   
2. Git clone the git repository
    ```
   $ git clone https://github.com/HybridRobotics/dynamics_on_manifolds.git
   ```

### Running Examples

#### Windows

 - TODO (Makes use of the IntelliJ IDE)

#### Ubuntu 

##### Using command line

1.  `cd` into `dynamics_on_manifolds`
    ```
    $ cd <path to the package location>/dynamics_on_manifolds
    ``` 
2. Run `sbt`. This will open up the sbt console. Find full details [here](https://docs.scala-lang.org/getting-started/sbt-track/getting-started-with-scala-and-sbt-on-the-command-line.html). 
    ```
    $ sbt
    ``` 
   It would take a minute or two to load the sbt console. 
  
3. Type `~run` in the sbt console. The ~ is optional and causes sbt to re-run on every file save, allowing for a fast edit/run/debug cycle. sbt will also generate a target directory which you can ignore.
    ```
   sbt:dynamics_on_manifolds> ~run
   ```
   Build might take 1-2 mins. After you build your project, sbt will create more target directories for generated files. You can ignore these.

4. `~run` runs the `./src/main/scala/hybridrobotics/dynamics/Main.scala` in the package. Code for `Main.scala` is given below,
    
    ```$xslt
    package hybridrobotics.dynamics    
    import hybridrobotics.dynamics.examples._
    
    object Main extends App {
    
      mechanical_systems.SphericalPendulum.main() // Nonlinear dynamics
      variation_linearization.RigidPendulum.main() // Variation-based linearization
    
    }
    ```
5. Matlab and Latex files are saved to `./output/matlab` and `./output/latex` folders respectively. 

##### Using Intellij IDE 
 
 - Same as Windows instructions

You have now successfully set up your computational environment and are ready for modeling. Located in the dynamics_on_manifolds package (`.src/main/scala/hybridrobotics/dynamics/examples`) are a set of starter files, which involve configuration setups for a variety of mechanical systems. They include:

| Nonlinear dynamics  (`mechanical_systems`)     | Variation based linearization (`variation_linearization`) |
| ----------- | ----------- |
| `SphericalPendulum.scala`         | `PointMass.scala`       |
| `DoubleSphericalPendulum.scala`   | `RigidPendulum.scala`        |
| `Pendulum3D.scala`                | `SphericalPendulum.scala`       |
| `DoublePendulum3D.scala`          | `QuadrotorWithLoad.scala`        |
| `ThreeLinkWalker.scala`           | `MultipleQuadrotorRigidPayload.scala`      |
| `ThreeLinkWalkerFlight.scala`     |  |
| `QuadrotorWithLoad.scala`         |  |
| `QuadrotorWithContinuum.scala`    |  |


### Generating dynamics for custom mechanical systems

-  Full documentation for `dynamics_on_manifold` package can be found in the [https://github.com/HybridRobotics/dynamics_on_manifolds/wiki](https://github.com/HybridRobotics/dynamics_on_manifolds/wiki). 

- Template example for generating nonlinear dynamics is [here](). 

- Template example for computing variation based linearization dynamics, [link](). 


