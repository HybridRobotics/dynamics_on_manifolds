// PrintTree.scala
package hybridrobotics.dynamics.operations

import java.io.{File, PrintWriter}

import hybridrobotics.dynamics.operations.Collection._
import hybridrobotics.dynamics.operations.Separation._
import hybridrobotics.dynamics.operations.Simplification._

object PrintLine {

  def checkSymbols(s_ugly: String): String = {
    var s = s_ugly
    val ddt = s contains "dotdot"
    val dt = s contains "dot"
    val l = s.length

    if (ddt) {
      s = "\\ddot{" + s.slice(0, l - 6) + "}"
    }
    else if (dt) {
      s = "\\dot{" + s.slice(0, l - 3) + "}"
    }
    else {
      s = s
    }

    if (s contains "theta") {
      s.replace("theta", "\\theta ")
    }
    else if (s contains "omega") {
      s.replace("omega", "\\omega ")
    }
    else if (s contains "Omega") {
      s.replace("Omega", "\\Omega ")
    }
    else if (s contains "eta") {
      s.replace("eta", "\\eta ")
    }
    else if (s contains "xi") {
      s.replace("xi", "\\xi ")
    }
    else {
      s
    }
  }

  // Print Matrix to Screen
  def printMLatex(m: MExp): String = m match {
    case SMMul(a, b) => printMLatex(a) + " " + printLatex(b)
    case MAdd(a, b) => "(" + printMLatex(a) + "+" + printMLatex(b) + ")"
    case Mat(s) => checkSymbols(s)
    case SMat(s) => checkSymbols(s)
    case CSMat(s) => checkSymbols(s)
    case CMat(s) => checkSymbols(s)
    case SkewMat(s) => "\\hatmap{" + checkSymbols(s) + "}"
    case MMul(s, r) => printMLatex(s) + " " + printMLatex(r)
    case deltaM(s) => "\\delta " + printMLatex(s)
    case transpose(s) => printMLatex(s) + "^{T}"
  }

  // Print Vector to Screen
  def printVLatex(v: VExp): String = v match {
    case deltaV(u) => "\\delta " + printVLatex(u)
    case Vec(s) => checkSymbols(s)
    case UVec(s) => checkSymbols(s)
    case CVec(s) => checkSymbols(s)
    case VAdd(u, v) => printVLatex(u) + "+" + printVLatex(v)
    case Cross(u, v) => printVLatex(u) + "\\times " + printVLatex(v)
    case SMul(u, v) => printVLatex(u) + " " + printLatex(v)
    case VMul(u, v) => printMLatex(u) + " " + printVLatex(v)
    case ZVec(s) => checkSymbols(s)
  }

  // Print Scalar to Screen
  def printLatex(e: Exp): String = e match {
    case deltaS(u) => "\\delta" + "(" + printLatex(u) + ")"
    case Var(s) => checkSymbols(s)
    case Cons(s) => checkSymbols(s)
    case Num(d) => "(" + d.toString + ")"
    case Add(u, v) => printLatex(u) + "+" + printLatex(v)
    case Mul(deltaS(u), v) => printLatex(deltaS(u)) + "* (" + printLatex(v) + ")"
    case Mul(u, v) => printLatex(u) + " " + printLatex(v)
    case Dot(u, v) => printVLatex(u) + "\\cdot " + printVLatex(v)
    case Par(u) => "(" + printLatex(u) + ")"
  }

  // Print Matrix to Screen
  def printMTree(m: MExp): String = m match {
    case SMMul(a, b) => printMTree(a) + "*" + printTree(b)
    case MAdd(a, b) => "(" + printMTree(a) + "+" + printMTree(b) + ")"
    case Mat(s) => s
    case SMat(s) => s
    case CSMat(s) => s
    case CMat(s) => s
    case SkewMat(s) => s
    case MMul(s, r) => printMTree(s) + "*" + printMTree(r)
    case deltaM(s) => "deltaV(" + printMTree(s) + ")"
    case transpose(s) => "transpose(" + printMTree(s) + ")"
  }

  // Print Vector to Screen
  def printVTree(v: VExp): String = v match {
    case deltaV(u) => "delta" + "(" + printVTree(u) + ")"
    case Vec(s) => s
    case UVec(s) => s
    case CVec(s) => s
    case VAdd(u, v) => printVTree(u) + "+" + printVTree(v)
    case Cross(u, v) => printVTree(u) + "x" + printVTree(v)
    case SMul(u, v) => printVTree(u) + "*" + printTree(v)
    case VMul(u, v) => printMTree(u) + "*" + printVTree(v)
    case ZVec(s) => s
  }

  // Print Scalar to Screen
  def printTree(e: Exp): String = e match {
    case deltaS(u) => "delta" + "(" + printTree(u) + ")"
    case Var(s) => s
    case Cons(s) => s
    case Num(d) => d.toString
    case Add(u, v) => printTree(u) + "+" + printTree(v)
    case Mul(deltaS(u), v) => printTree(deltaS(u)) + "* (" + printTree(v) + ")"
    case Mul(u, v) => printTree(u) + "*" + printTree(v)
    case Dot(u, v) => printVTree(u) + "." + "(" + printVTree(v) + ")"
    case Par(u) => "(" + printTree(u) + ")"
  }


  // Prints set of Equations of Motion
  def printEOM(e: Exp, s: List[Exp], v: List[VExp]): Tuple3[List[String], List[Exp], List[VExp]] = {
    var eom_latex: List[String] = List()
    var piece: Exp = Num(0)
    var counter: Int = 0
    var eom_scalar: List[Exp] = List()
    var eom_vector: List[VExp] = List()

    // Extracts one equation for each collection term
    for (scalar <- s) {
      piece = colScalar(e, scalar)
      piece = separate(piece, scalar)
      val line = simplify(piece)
      val finalOutput = removeOneCoefs(line)
      eom_latex = eom_latex :+ "Lag" + (counter + 1).toString + ":$" + printLatex(finalOutput) + " = 0$"
      eom_scalar = eom_scalar :+ finalOutput
      counter = counter + 1
    }

    // Extracts one equation for each collection term
    for (vector <- v) {
      piece = col(e, vector)
      piece = separateV(piece, vector)
      val prep = simplify(piece)

      val line = extractAll(prep)

      val finalOutput = removeOneCoefsV(line)
      eom_latex = eom_latex :+ "Lag" + (counter + 1).toString + ": $" + printVLatex(finalOutput) + " = 0$"
      eom_vector = eom_vector :+ finalOutput
      counter = counter + 1
    }
    return Tuple3(eom_latex, eom_scalar, eom_vector)
  }

  def print2LatexFile(equations: List[String], fileName: String): Unit = {
    val FILE_PATH = new java.io.File(".").getCanonicalPath
    val writer = new PrintWriter(new File(FILE_PATH + File.separator+ "output"+File.separator + fileName + ".tex"))
    writer.write(fileName + ": Equations of Motion\n\\begin{itemize}\n")
    for (str <- equations) {
      writer.write("""\item """ + str + "\n")
    }
    writer.write("""\end{itemize}""")
    writer.close()
  }

}