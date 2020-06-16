package hybridrobotics.dynamics.calculus

import hybridrobotics.dynamics.data_types._
//import hybridrobotics.dynamics.calculus.ApplyRules._
//import hybridrobotics.dynamics.calculus.Differentiation._
//import hybridrobotics.dynamics.calculus.PrintLine._

object SystemGeometry {

  //  def HatMap(v: Any): MatrixExpr = v match {
  //    case v: String => SkewMat(v)
  //    case v: Vec => SkewMat(v.s)
  //    case v: UVec => SkewMat(v.s)
  //    case v: ZVec => SkewMat(v.s)
  //    case v: AVec => SkewMat(v.s)
  //    case SkewMatVec(m) => SkewMat(m)
  //  }
  //
  //  def VeeMap(m: Any): VectorExpr = m match {
  //    case m: String => SkewMatVec(m)
  //    case m: Mat => SkewMatVec(m.s)
  //    case m: SMat => SkewMatVec(m.s)
  //    case m: CSMat => SkewMatVec(m.s)
  //    case m: CMat => SkewMatVec(m.s)
  //    case SkewMat(s) => Vec(s)
  //  }

  //  def VeeMap(m: MatrixExpr): VectorExpr = {
  //    val v
  //  }

  //  def getXi(v: VectorExpr): VectorExpr = v match {
  //    case UVec(s) => Vec(s.replace("q", "xi"))
  //    case u: VectorExpr => deltaV(u)
  //  }
  //
  //  def getEta(m: MatrixExpr): VectorExpr = m match {
  //    case Mat(s) => Vec(s.replace("R", "eta"))
  //    case m: MatrixExpr => Vec("none")
  //  }
  //
  //  def getSkewEta(m: MatrixExpr): MatrixExpr = m match {
  //    case Mat(s) => SkewMat(s.replace("R", "eta"))
  //    case m: MatrixExpr => m
  //  }
  //
  //  def getOmega(m: MatrixExpr): VectorExpr = m match {
  //    case Mat(s) => Vec(s.replace("R", "Omega"))
  //    case m: MatrixExpr => Vec("None")
  //  }

  def getSO3Variation(matrixExpr: MatrixExpr): VectorExpr = {
    matrixExpr match {
      case matrixExpr: SO3 => matrixExpr.getVariation
    }
  }
  def getMatrixVariation(expr: MatrixExpr): MatrixExpr = {
    expr match {
      case expr: MatrixExpr => expr.delta()
    }
  }

  def generateCollectionTerms(configVars: Tuple3[List[ScalarExpr], List[VectorExpr], List[MatrixExpr]]):
  Tuple3[List[ScalarExpr], List[VectorExpr], List[MatrixExpr]] = {
    var colTermsScalars: List[ScalarExpr] = List()
    var colTermsVectors: List[VectorExpr] = List()
    var colTermsMatrices: List[MatrixExpr] = List()
    var scalars = configVars._1
    var vectors = configVars._2
    var matrices = configVars._3
    for (s <- scalars) {
      colTermsScalars = colTermsScalars :+ s.getVariation
    }
    for (v <- vectors) {
      colTermsVectors = colTermsVectors :+ v.getVariation
    }
    for (m <- matrices) {
      if (m.isInstanceOf[SO3]) colTermsVectors = colTermsVectors :+ getSO3Variation(m)
      else colTermsMatrices = colTermsMatrices :+ getMatrixVariation(m)
    }
    return Tuple3(colTermsScalars, colTermsVectors, colTermsMatrices)
  }

  //  def generateRules(configVars: Tuple3[List[ScalarExpr], List[VectorExpr], List[MatrixExpr]], colTerms: Tuple3[List[ScalarExpr], List[VectorExpr], List[MatrixExpr]]): Tuple3[Map[ScalarExpr, ScalarExpr], Map[VectorExpr, VectorExpr], Map[MatrixExpr, MatrixExpr]] = {
  //    var rules1: Map[ScalarExpr, ScalarExpr] = Map()
  //    var rules2: Map[VectorExpr, VectorExpr] = Map()
  //    var rules3: Map[MatrixExpr, MatrixExpr] = Map()
  //    var scalars = configVars._1
  //    var vectors = configVars._2
  //    var matrices = configVars._3
  //
  //    for (v <- vectors) {
  //      if (v.isInstanceOf[UnitVector]) {
  //        /* Adding the variation rules for unit vectors
  //            q.\delta q  = q dot cross(xi,q) = 0
  //            q.\dot{q}   = q dot cross(omega, q) = 0
  //            xi.q = 0
  //        */
  //        rules1 += (Dot(v, deltaV(v)) -> Num(0), Dot(v, diffV(v)) -> Num(0), Dot(getXi(v), v) -> Num(0))
  //
  //        /* Adding unit vector relation to its varation
  //            \delta q = cross(xi, q)
  //            \delta\dot q = cross(\dot \xi,q)+cross(xi, \dot q)
  //         */
  //        rules2 += (deltaV(v) -> Cross(getXi(v), v), deltaV(diffV(v)) -> VAdd(Cross(diffV(getXi(v)), v), Cross(getXi(v), diffV(v))))
  //      }
  //    }
  //    for (r <- matrices) {
  //      rules2 += deltaV(getOmega(r)) -> VAdd(Cross(getOmega(r), getEta(r)), diffV(getEta(r)))
  //      rules3 += deltaM(r) -> MMul(r, getSkewEta(r))
  //    }
  //    return Tuple3(rules1, rules2, rules3)
  //  }
  //
  //  // list equations of motion which are functions of configuration space -> S^2, SO(3)
  //  def checkGeometry(configVars: Tuple3[List[ScalarExpr], List[VectorExpr], List[MatrixExpr]], Rules: Tuple3[Map[ScalarExpr, ScalarExpr], Map[VectorExpr, VectorExpr], Map[MatrixExpr, MatrixExpr]]): Tuple3[Map[ScalarExpr, ScalarExpr], Map[VectorExpr, VectorExpr], Map[MatrixExpr, MatrixExpr]] = {
  //
  //    var g_seom: Map[ScalarExpr, ScalarExpr] = Map()
  //    var g_veom: Map[VectorExpr, VectorExpr] = Map()
  //    var g_meom: Map[MatrixExpr, MatrixExpr] = Map()
  //
  //    var vectors = configVars._2
  //    for (v <- vectors) {
  //      if ((applyConstraints(v dot diffV(v), Rules._1, Rules._2, Rules._3) == Num(0.0)) && (applyConstraints(v dot diffV(v), Rules._1, Rules._2, Rules._3) == Num(0.0))) {
  //        g_veom += (diffV(v) -> Cross(Vec("omega" + printVLatex(v).replaceFirst("q", "")), v))
  //      }
  //    }
  //
  //    var matrices = configVars._3
  //    for (m <- matrices) {
  //      g_meom += (MMul(m, SkewMat("Omega" + printMLatex(m).replaceFirst("R", ""))) -> diffM(m))
  //    }
  //
  //    var geometrical_eoms = Tuple3(g_seom, g_veom, g_meom)
  //
  //    return geometrical_eoms
  //  }
}
