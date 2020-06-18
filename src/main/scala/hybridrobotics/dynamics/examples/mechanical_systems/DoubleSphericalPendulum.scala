package hybridrobotics.dynamics.examples.mechanical_systems

import hybridrobotics.dynamics.operations.Differentiation.diffV
import hybridrobotics.dynamics.operations.DynamicalModelComputation.computeEquationsOfMotion
import hybridrobotics.dynamics.operations._

object DoubleSphericalPendulum
{
def main()
{
// define constant scalars
val g = Cons("g")  // gravitational constant
val m1 = Cons("m1")  // point mass
val m2 = Cons("m2")  // point mass
val l1 = Cons("l1")  // length at which point mass hangs
val l2 = Cons("l2")  // length at which point mass hangs

// define vectors
val e3   = CVec(  "e3")  // constant vector, points in direction of positive potential energy
val  q1  = UVec(  "q1")  // unit vector, points from pendulum attachment point to location of hanging mass
val  q2  = UVec(  "q2")  // unit vector, points from pendulum attachment point to location of hanging mass
val xi1  =  Vec( "xi1")  // vector orthoganal to q and dq
val xi2  =  Vec( "xi2")  // vector orthoganal to q and dq

// set configuration variables
val configVars = Tuple3(List(),List(q1,q2),List())

// define lagrangian
val KE = (Num(0.5) * m1 * l1 * l1 * (diffV(q1) dot diffV(q1))) + (Num(0.5) * m2 * l1 * l1 * (diffV(q1) dot diffV(q1))) + (m2 * l1 * l2 * (diffV(q1) dot diffV(q2))) + (Num(0.5) * m2 * l2 * l2 * (diffV(q2) dot diffV(q2)))
val PE = ((m1+m2) * g * l1 * (q1 dot e3)) + (m2 * g * l2 * (q2 dot e3))
val L = KE - PE

// specify infinitesimal virtual work of system
val u1 = Vec("u1")
val u2 = Vec("u2")
val infWork = Dot(deltaV(q1),u1) + Dot(deltaV(q2),u2)

var eoms = computeEquationsOfMotion(L, infWork, configVars)
}
}
