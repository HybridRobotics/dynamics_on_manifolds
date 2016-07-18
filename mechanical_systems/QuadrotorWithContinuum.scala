// QuadrotorWithContinuum.scala
package dynamics_on_manifolds

import Differentiation._
import DynamicalModelComputation._
import Variation._

object QuadrotorWithContinuum
{

  def main()
  {

    // define constant scalars
    val g = Cons("g")
    val mq = Cons("mq")
    val ma = Cons("ma")
    val mi = Cons("mi")
    val li = Cons("li")      
    val lj = Cons("lj")      

    // define vectors
    var e3    = CVec( "e3")
    var J     = CSMat("J")
    var eta   = Vec(  "eta")
    var omega = Vec(  "omega")
    var xdot = Vec("xdot")
    var x    = Vec("x")
    var qi    = UVec("qi")
	var qidot = diffV(qi)
    var qj    = UVec("qj")
	var qjdot = diffV(qj)
    var xii = Vec("xii")
    var xij = Vec("xij")


    // define matrices
    var eta_skew   = SkewMat("eta")
    var R   = Mat("R")

    // set configuration variables
    val configVars = Tuple3(List(), List(qi), List(R))

    // define lagrangian
    val KE = Num(0.5)*(mq+mi)*Dot(xdot,xdot) + ma*li*Dot(xdot,qidot) + Num(0.5)*ma*li*lj*Dot(qidot,qjdot) + Num(0.5)*Dot(omega,J**omega)
    val PE = (mq + mi)*g*Dot(x,e3) - ma*g*li*Dot(qi,e3)

    var L = KE - PE

    // define infi
    val fi = Mat("fi")
    val M = Vec("M")
    val infWork = (eta dot M)

    computeEquationsOfMotion(L, infWork, configVars)
    }
}
