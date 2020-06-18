package hybridrobotics.dynamics.examples.mechanical_systems

import hybridrobotics.dynamics.operations._
import hybridrobotics.dynamics.operations.Variation.delta
import hybridrobotics.dynamics.operations.SystemGeometry.{HatMap, VeeMap}
import hybridrobotics.dynamics.operations.Differentiation.diff
object BasicGeometry {

  class myVar (s:String) {
    //Wrap s:String to MExp
    import language.implicitConversions
    implicit def str2ExampleClass(s: String, t: String): myVar = MmyVar(s)

  }
  case class MmyVar(s:String) extends myVar(s)
  case class UmyVar(s:String) extends myVar(s)

  def main () : Unit  = {
//
    val M = Mat("M")
    val R = SO3("R")
    val eta = R.getEta
    val deltaR = delta(R)
    val deltaM1 = delta(M)
    val delatM2 = deltaM(M)
    println(s""+R.s+"")

    val dotR = diff(R)
    val hateta = HatMap(eta)
    val back2eta = VeeMap(hateta)
//
//    val a1 = MmyVar("sample")
//    val a2 = UmyVar("sample")
//
//    def testFunc(a: myVar): myVar = a match {
//      case a: MmyVar => MmyVar(a.s+"MmyVar")
//      case a: UmyVar => UmyVar(a.s+"UmyVar")
//    }
//
//    val a11 = testFunc(a1)
//    val a12 = testFunc(a2)

    println("Done!!!")

  }

}
