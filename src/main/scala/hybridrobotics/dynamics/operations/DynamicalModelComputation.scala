package hybridrobotics.dynamics.operations

import java.io.{File, PrintWriter}

import ApplyRules._
import Cancellation._
import Differentiation._
import Expansion._
import IntegrationByParts._
import PrintLine._
import Simplification._
import SystemGeometry._
import Variation._

object DynamicalModelComputation {
  def computeEquationsOfMotion(Lagrangian: Exp, infWork: Exp, configVars: Tuple3[List[Exp], List[VExp], List[MExp]]): Tuple4[List[String], List[Exp], List[VExp], List[MExp]] = {
    // TODO output both kinematic (Euclidean and geometric) and dynamics equations

    val startTime = System.nanoTime() // track computation time

    // Convert Input Tree to Desired Format (terms expanded and combined numeric constants)
    var L = simp(Lagrangian)
    L = expansion(L)
    L = simplify(L)

    // Step 0: Generate Collection Terms and Constraints of the Configuration Manifold
    // Check for Geometry Dependent Equations of Motion

    // collects the variations
    // List(scalar variations, vector variations, matrix variations)
    var colTerms = generateCollectionTerms(configVars)

    // collect various variation and manifold kinematic rules
    // conditions on q and R
    var Rules = generateRules(configVars, colTerms)

    // generating kinematic equations for S2 and SO3 // TODO much of this can be avoided
    var geometric_eom = checkGeometry(configVars, Rules)
    var rules2 = Rules._2
    // for (keyVal <- geometric_eom._2) {rules2 += keyVal}
    var rules3 = Rules._3
    // for (keyVal <- geometric_eom._3) {rules3 += keyVal}

    // Step 1 in the readme Diagram has already been completed (lagrangian provided)

    // Step 2: Take variation of the lagrangian
    var dL = variation(L) + infWork
    dL = simplify(dL)

    // Step 3: Substitute in for Kinematic Constraints and Variations of the System
    dL = applyConstraints(dL, Rules._1, rules2, rules3)


    // Set up Arrays of Collection Terms
    val vectors = colTerms._2
    var vectordots: List[VExp] = List()
    for (vector <- vectors) {
      vectordots = vectordots :+ diffV(vector)
    }
    val scalars = colTerms._1
    var scalardots: List[Exp] = List()
    for (s <- scalars) {
      scalardots = scalardots :+ diffS(s)
    }

    // Step 4 and 5: Apply Integration By Parts to the Collection Terms
    dL = expansion(dL)
    dL = integrateByPartsV(dL, vectordots) // collects for each vectordot (step 4) dot before ibp
    dL = integrateByParts(dL, scalardots) // collects for each scalardot (step 4) dot before ibp

    // Simplify Tree (parallel vector cross products, term cancellation)
    dL = expansion(dL)
    dL = applyConstraints(dL, Rules._1, Rules._2, Rules._3)
    dL = expansion(dL)
    dL = simplify(dL)
    // Organizing the tree terms alphabetically preconditions the tree for cancelling extra terms.
    dL = org(dL)
    var split_dL: Tuple2[List[Exp], List[Exp]] = split(dL)
    dL = cancelTerms(split_dL._1, split_dL._2)

    dL = expansion(dL)
    dL = simplify(dL)

    // TODO generate dynamics as M(q,dq)ddq + C(q,dq) + B = J^TF
    // TODO additionally format dynamics as \dot{x} = f(x,u) or f(x) + g(x)u

    // Step 6 and 7: Collect for each term and extract the equations of motion.
    val eom: Tuple3[List[String], List[Exp], List[VExp]] = printEOM(dL, scalars, vectors)
    var eom_latex = eom._1

    // Consolidate geometrical dependent and variation dependent equations of motion.
    var s_eom = eom._2
    for (keyVal <- geometric_eom._1) {
      var geom = Add(Mul(keyVal._1, Num(-1.0)), keyVal._2)
      s_eom = s_eom :+ geom
      eom_latex = eom_latex :+ "Geo: $" + printLatex(geom) + " = 0$"
    }

    var v_eom = eom._3
    for (keyVal <- geometric_eom._2) {
      var geom = VAdd(SMul(keyVal._1, Num(-1.0)), keyVal._2)
      v_eom = v_eom :+ geom
      eom_latex = eom_latex :+ "Geo: $" + printVLatex(geom) + " = 0$"
    }

    var m_eom: List[MExp] = List()
    for (keyVal <- geometric_eom._3) {
      var geom = MAdd(SMMul(keyVal._1, Num(-1.0)), keyVal._2)
      m_eom = m_eom :+ geom
      eom_latex = eom_latex :+ "Geo: $" + printMLatex(geom) + " = 0$"
    }

    val endTime = System.nanoTime()
    println("ComputationTime:" + (endTime - startTime) / 1000000)

    // Tuple4[List[String], List[Exp], List[VExp], List[MExp]]
    return Tuple4(eom_latex, s_eom, v_eom, m_eom)
  }


  // Variation based linearization
  def computeVariationLinearization(): Unit = {

  }
}
