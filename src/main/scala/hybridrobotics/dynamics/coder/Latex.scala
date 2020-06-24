package hybridrobotics.dynamics.coder

import java.io.{File, PrintWriter}
import hybridrobotics.dynamics.data_types._

object Latex {

  def checkSymbols(s_ugly: String): String = {
    var s = s_ugly
    val ddt = s contains "dotdot"
    val dt = s contains "dot"
    val l = s.length

    if (ddt) {
      s = "\\ddot{" + s.slice(6, l) + "}"
    }
    else if (dt) {
      s = "\\dot{" + s.slice(3, l) + "}"
    }
    else {
      s = s
    }

    if (s contains "theta") {
      s.replace("theta", "\\theta")
    }
    else if (s contains "omega") {
      s.replace("omega", "\\omega")
    }
    else if (s contains "Omega") {
      s.replace("Omega", "\\Omega")
    }
    else if (s contains "eta") {
      s.replace("eta", "\\eta")
    }
    else if (s contains "xi") {
      s.replace("xi", "\\xi")
    }
    else {
      s
    }
  }

  // Print Matrix to Screen
  def printMLatex(m: MatrixExpr): String = m match {
    case SMMul(a, b) => printLatex(b) + " " + printMLatex(a)
    case MAdd(a, b) => "(" + printMLatex(a) + "+" + printMLatex(b) + ")"
    case SO3(s) => checkSymbols(s)
    case Matrix(s) => checkSymbols(s)
    case SymMatrix(s) => checkSymbols(s)
    case ConstSymMatrix(s) => checkSymbols(s)
    case ZeroMatrix() => "O"
    case IdentityMatrix() => "I"
    case ConstMatrix(s) => checkSymbols(s)
    case SkewSymMatrix(s) => "\\hatmap{" + checkSymbols(s) + "}"
    case MMul(s, r) => printMLatex(s) + " " + printMLatex(r)
    case VVMul(u, v) => "(" + printVLatex(u) + " " + printVLatex(v) + ")"
    case DeltaM(s) => "\\delta " + printMLatex(s)
    case TransposeMatrix(s) => "{(" + printMLatex(s) + ")}^{T}"
    case CrossMap(v) => "{(" + printVLatex(v) + ")}^\\times"
    case _ => "missingMatrixChar"
  }

  // Print Vector to Screen
  def printVLatex(v: VectorExpr): String = v match {
    case DeltaV(u) => "\\delta " + printVLatex(u)
    case Vector(s) => checkSymbols(s)
    case UnitVector(s) => checkSymbols(s)
    case ConstVector(s) => checkSymbols(s)
    case S2(s) => checkSymbols(s)
    case VAdd(u, v) => "(" + printVLatex(u) + "+" + printVLatex(v) + ")"
    case Cross(u, v) => "(" + printVLatex(u) + "\\times " + printVLatex(v) + ")"
    case SMul(u, v) => printLatex(v) + printVLatex(u)
    case MVMul(u, v) => printMLatex(u) + " " + printVLatex(v)
    case TransposeVector(v) => "{(" + printVLatex(v) + ")}^{T}"
    case ZeroVector() => "0_{3,1}"
    case OnesVector() => "1_{3,1}"
    case _ => "missingVectorChar"
  }

  // Print Scalar to Screen
  def printLatex(e: ScalarExpr): String = e match {
    case DeltaS(u) => "( \\delta " + printLatex(u) + ")"
    case VarScalar(s) => checkSymbols(s)
    case ConstScalar(s) => checkSymbols(s)
    case NumScalar(d) => "(" + d.toString + ")"
    case Add(u, v) => "(" + printLatex(u) + "+" + printLatex(v) + ")"
    case Mul(DeltaS(u), v) => printLatex(DeltaS(u)) + "* (" + printLatex(v) + ")"
    case Mul(u, v) => printLatex(u) + " " + printLatex(v)
    case Dot(u, v) => "\\big((" + printVLatex(u) + ")\\cdot( " + printVLatex(v) + ")\\big)"
    case Par(u) => "(" + printLatex(u) + ")"
    case _ => "missingScalarChar"
  }

  // Print Matrix to Screen
  def printMTree(m: MatrixExpr): String = m match {
    case SMMul(a, b) => printMTree(a) + "*" + printTree(b)
    case MAdd(a, b) => "(" + printMTree(a) + "+" + printMTree(b) + ")"
    case Matrix(s) => s
    case SymMatrix(s) => s
    case ConstSymMatrix(s) => s
    case ConstMatrix(s) => s
    case SkewSymMatrix(s) => s
    case MMul(s, r) => printMTree(s) + "*" + printMTree(r)
    case DeltaM(s) => "DeltaV(" + printMTree(s) + ")"
    case TransposeMatrix(s) => "transpose(" + printMTree(s) + ")"
  }

  // Print Vector to Screen
  def printVTree(v: VectorExpr): String = v match {
    case DeltaV(u) => "delta" + "(" + printVTree(u) + ")"
    case Vector(s) => s
    case UnitVector(s) => s
    case ConstVector(s) => s
    case ZeroVector() => "0_{3,1}"
    case OnesVector() => "1_{3,1}"
    case VAdd(u, v) => printVTree(u) + "+" + printVTree(v)
    case Cross(u, v) => printVTree(u) + "x" + printVTree(v)
    case SMul(u, v) => printVTree(u) + "*" + printTree(v)
    case MVMul(u, v) => printMTree(u) + "*" + printVTree(v)
    //    case ZVec(s) => s
  }

  // Print Scalar to Screen
  def printTree(e: ScalarExpr): String = e match {
    case DeltaS(u) => "delta" + "(" + printTree(u) + ")"
    case VarScalar(s) => s
    case ConstScalar(s) => s
    case NumScalar(d) => d.toString
    case Add(u, v) => printTree(u) + "+" + printTree(v)
    case Mul(DeltaS(u), v) => printTree(DeltaS(u)) + "* (" + printTree(v) + ")"
    case Mul(u, v) => printTree(u) + "*" + printTree(v)
    case Dot(u, v) => "(" + printVTree(u) + ")\\cdot(" + printVTree(v) + ")"
    case Par(u) => "(" + printTree(u) + ")"
  }


  //  // Prints set of Equations of Motion
  //  def printEOM(e: ScalarExpr, s: List[ScalarExpr], v: List[VectorExpr]): Tuple3[List[String], List[ScalarExpr], List[VectorExpr]] = {
  //    var eom_latex: List[String] = List()
  //    var piece: ScalarExpr = Num(0)
  //    var counter: Int = 0
  //    var eom_scalar: List[ScalarExpr] = List()
  //    var eom_vector: List[VectorExpr] = List()
  //
  //    // Extracts one equation for each collection term
  //    for (scalar <- s) {
  //      piece = colScalar(e, scalar)
  //      piece = separate(piece, scalar)
  //      val line = simplify(piece)
  //      val finalOutput = removeOneCoefs(line)
  //      eom_latex = eom_latex :+ "Lag" + (counter + 1).toString + ":$" + printLatex(finalOutput) + " = 0$"
  //      eom_scalar = eom_scalar :+ finalOutput
  //      counter = counter + 1
  //    }
  //
  //    // Extracts one equation for each collection term
  //    for (vector <- v) {
  //      piece = col(e, vector)
  //      piece = separateV(piece, vector)
  //      val prep = simplify(piece)
  //
  //      val line = extractAll(prep)
  //
  //      val finalOutput = removeOneCoefsV(line)
  //      eom_latex = eom_latex :+ "Lag" + (counter + 1).toString + ": $" + printVLatex(finalOutput) + " = 0$"
  //      eom_vector = eom_vector :+ finalOutput
  //      counter = counter + 1
  //    }
  //    return Tuple3(eom_latex, eom_scalar, eom_vector)
  //  }

  def variationCoeffs2LatexEquation(coefficients: (Map[ScalarExpr, VectorExpr], Map[VectorExpr, MatrixExpr], Map[MatrixExpr, MatrixExpr]) ): String = {
    var latexEquation: String = "Variation Coefficients extracted\n\\begin{gather}\n"
    for ((k, v) <- coefficients._2) {
      val ks: String = printVLatex(k)
      val vs: String = printMLatex(v)
      latexEquation = latexEquation + "\\Big[" + vs + "\\Big]" + ks + "+\\nonumber\\\\\n"
    }
    latexEquation = latexEquation + "=0\n\\end{gather}"
    //    print("%s\n", latexEquation)
    return latexEquation
  }

  def print2LatexFile(equations: List[String], fileName: String): Unit = {
    val FILE_PATH = new java.io.File(".").getCanonicalPath
    val directory = new File(FILE_PATH + File.separator + "output" + File.separator + "latex")
    directory.mkdirs()
    val writer = new PrintWriter(new File(directory+ File.separator + fileName + ".tex"))
    writer.write(fileName + ": Equations of Motion\n\\begin{itemize}\n")
    for (str <- equations) {
      writer.write("""\item """ + str + "\n")
    }
    writer.write("""\end{itemize}""")
    writer.close()
  }

}
