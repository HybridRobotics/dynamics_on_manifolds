// DynamicalModelComputation.scala
package dynamics_on_manifolds

import ApplyRules._
import Cancellation._
import Differentiation._
import Expansion._
import IntegrationByParts._
import PrintLine._
import Separation._
import Simplification._
import SystemGeometry._
import Variation._

object DynamicalModelComputation
{


  def computeEquationsOfMotion( Lagrangian:Exp, infWork:Exp, configVars:Tuple3[List[Exp],List[VExp],List[MExp]]) : Tuple3[List[Exp],List[VExp],List[MExp]] =
  {
    val startTime = System.nanoTime() // track computation time
    
    // Convert Input Tree to Desired Format (terms expanded and combined numeric constants)
    var L = simp(Lagrangian)
	L = expansion(L)
    L = simplify(L)
      
	// Step 0: Generate Collection Terms and Constraints of the Configuration Manifold
    // Check for Geometry Dependent Equations of Motion
    var colTerms = generateCollectionTerms(configVars)
    var Rules = generateRules(configVars,colTerms)
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
    var vectordots:List[VExp] = List()
    for (vector <- vectors) {vectordots = vectordots :+ diffV(vector)}
    val scalars = colTerms._1
    var scalardots:List[Exp] = List()
    for (s <- scalars) {scalardots = scalardots :+ diffS(s)}
	
    // Step 4 and 5: Apply Integration By Parts to the Collection Terms
    dL = expansion(dL)

    dL = integrateByPartsV(dL, vectordots) // collects for each vectordot (step 4) dot before ibp

    dL = integrateByParts(dL, scalardots)  // collects for each scalardot (step 4) dot before ibp


    // Simplify Tree (parallel vector cross products, term cancellation)
    dL = expansion(dL)
    dL = applyConstraints(dL,Rules._1, Rules._2, Rules._3)
    dL = expansion(dL)
    dL = simplify(dL)
    // Organizing the tree terms alphabetically preconditions the tree for cancelling extra terms.
    dL = org(dL)
    var split_dL:Tuple2[List[Exp],List[Exp]] = split(dL)
    dL = cancelTerms(split_dL._1, split_dL._2)



    dL = expansion(dL)
    dL = simplify(dL)      
 
    // Step 6 and 7: Collect for each term and extract the equations of motion.
    var eom:Tuple3[String,List[Exp],List[VExp]] = printEOM(dL,scalars,vectors)
	println(eom._1)

	// Consolidate geometrical dependent and variation dependent equations of motion.

	var s_eom = eom._2
	for (keyVal <- geometric_eom._1) 
    {
		var geom = Add(Mul(keyVal._1,Num(-1.0)),keyVal._2)
		s_eom = s_eom :+ geom
    	println("Geo: " + printLatex(geom) + " = 0")
	}

	var v_eom = eom._3
	for (keyVal <- geometric_eom._2) 
    {
		var geom = VAdd(SMul(keyVal._1,Num(-1.0)),keyVal._2)
		v_eom = v_eom :+ geom
    	println("Geo: " + printVLatex(geom) + " = 0")
	}
	
	var m_eom:List[MExp] = List()
	for (keyVal <- geometric_eom._3) 
    {
		var geom = MAdd(SMMul(keyVal._1,Num(-1.0)),keyVal._2)
		m_eom = m_eom :+ geom
    	println("Geo: " + printMLatex(geom) + " = 0")
	}
	
    val endTime = System.nanoTime()
    println("ComputationTime:" + (endTime - startTime)/1000000)
	

	return Tuple3(s_eom,v_eom, m_eom)

  }
}