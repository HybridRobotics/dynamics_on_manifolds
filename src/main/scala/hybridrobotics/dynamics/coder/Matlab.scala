package hybridrobotics.dynamics.coder

import java.util.Date
import java.io.{File, PrintWriter}
import hybridrobotics.dynamics.data_types._

object Matlab {

  def generateLinearDynamics(coefficients: List[(Map[ScalarExpr, VectorExpr], Map[VectorExpr, MatrixExpr], Map[MatrixExpr, MatrixExpr])],
                             output_variables: List[Any], state_variables: List[Any], input_variables: List[Any]): Map[String, String] = {
    // This function generates M\ddot{x} = A*x + B*u function // TODO update this compute M\dot{x} = Ax + Bu

    // Dynamics
    var d2x: String = "["
    for (expr <- output_variables) {
      d2x = d2x + expr2Matlab(expr) + ";\n"
    }
    d2x = d2x + "];"

    // States
    var states: String = "["
    for (expr <- state_variables) {
      states = states + expr2Matlab(expr) + ";\n"
    }
    states = states + "];"

    // Inputs
    var inputs: String = "["
    for (expr <- input_variables) {
      inputs = inputs + expr2Matlab(expr) + ";\n"
    }
    inputs = inputs + "];"

    // Mass Matrix Equivalent
    var M: String = "["
    for (coeffs <- coefficients) {
      for (expr <- output_variables) {
        M = M + coefficient2Matlab(expr, coeffs) + ","
      }
      M = M + ";\n"
    }
    M = M + "];"

    // Linear Dynamics State Matrix: A
    var A: String = "-["
    for (coeffs <- coefficients) {
      for (expr <- state_variables) {
        A = A + coefficient2Matlab(expr, coeffs) + ","
      }
      A = A + ";\n"
    }
    A = A + "];"

    // Linear Dynamics Input Matrix: B
    var B: String = "-["
    for (coeffs <- coefficients) {
      for (expr <- input_variables) {
        B = B + coefficient2Matlab(expr, coeffs) + ","
      }
      B = B + ";\n"
    }
    B = B + "];"

    var LinearDynamics: Map[String, String] = Map()
    //    LinearDynamics += "type" -> "Linear"
    //    LinearDynamics += "dx" -> d2x
    //    LinearDynamics += "M" -> M
    //
    //    LinearDynamics += "states" -> states
    //    LinearDynamics += "inputs" -> inputs
    LinearDynamics += "A" -> A
    LinearDynamics += "B" -> B
    LinearDynamics
  }

  def coefficient2Matlab(expr: Any, coefficients: (Map[ScalarExpr, VectorExpr], Map[VectorExpr, MatrixExpr], Map[MatrixExpr, MatrixExpr])): String = {
    expr match {
      case expr: ScalarExpr => expr2Matlab(coefficients._1(expr))
      case expr: VectorExpr => expr2Matlab(coefficients._2(expr))
      case expr: MatrixExpr => expr2Matlab(coefficients._3(expr))
    }
  }

  def expr2Matlab(expr: Any): String = {
    expr match {
      case expr: MatrixExpr => matrixExpr2Matlab(expr)
      case expr: VectorExpr => vectorExpr2Matlab(expr)
      case expr: ScalarExpr => scalarExpr2Matlab(expr)
      case expr: Int => expr.toString
      case expr: Double => expr.toString
      case expr: String => expr
      case _ => "MISSING EXPR CASE"
    }
  }

  def matrixExpr2Matlab(mexpr: MatrixExpr): String = {
    mexpr match {
      // Matrix Functions
      case SMMul(a, b) => "(" + scalarExpr2Matlab(b) + ")*(" + matrixExpr2Matlab(a) + ")"
      case MAdd(a, b) => "((" + matrixExpr2Matlab(a) + ")+(" + matrixExpr2Matlab(b) + "))"
      case MMul(a, b) => "(" + matrixExpr2Matlab(a) + ")*(" + matrixExpr2Matlab(b) + ")"
      case VVMul(u, v) => "(" + vectorExpr2Matlab(u) + ")*(" + vectorExpr2Matlab(v) + ")"
      case TransposeMatrix(s) => "transpose(" + matrixExpr2Matlab(s) + ")"
      case CrossMap(v) => "hat(" + vectorExpr2Matlab(v) + ")"

      // Matlab Variables
      case ZeroMatrix() => "zeros(3,3)"
      case IdentityMatrix() => "eye(3)"
      case SO3(s) => getVariableName(s)
      case Matrix(s) => getVariableName(s)
      case SymMatrix(s) => getVariableName(s)
      case ConstMatrix(s) => getVariableName(s)
      case ConstSymMatrix(s) => getVariableName(s)
      case SkewSymMatrix(s) => getVariableName(s)
      case DeltaM(s) => "del_" + matrixExpr2Matlab(s)

      // Restricted functions/variables

      // default
      case _ => """error('MISSING MATRIX EXPR')"""
    }
  }

  def vectorExpr2Matlab(vexpr: VectorExpr): String = {
    vexpr match {
      // Vector operations
      case VAdd(u, v) => "((" + vectorExpr2Matlab(u) + ")+(" + vectorExpr2Matlab(v) + "))"
      case Cross(u, v) => "cross(" + vectorExpr2Matlab(u) + "," + vectorExpr2Matlab(v) + ")"
      case SMul(u, v) => "(" + scalarExpr2Matlab(v) + ")*(" + vectorExpr2Matlab(u) + ")"
      case MVMul(u, v) => "(" + matrixExpr2Matlab(u) + ")*(" + vectorExpr2Matlab(v) + ")"
      case TransposeVector(v) => "transpose(" + vectorExpr2Matlab(v) + ")"

      // Vector variables
      case OnesVector() => "ones(3,1)"
      case ZeroVector() => "zeros(3,1)"
      case S2(s) => getVariableName(s)
      case Vector(s) => getVariableName(s)
      case UnitVector(s) => getVariableName(s)
      case ConstVector(s) => getVariableName(s)
      case DeltaV(u) => "del_" + vectorExpr2Matlab(u)

      // Restricted functions/variables

      // default
      case _ => """error('MISSING VECTOR EXPR')"""
    }

  }

  def scalarExpr2Matlab(sexpr: ScalarExpr): String = {
    sexpr match {
      // scalar operations
      case Add(u, v) => "((" + scalarExpr2Matlab(u) + ")+(" + scalarExpr2Matlab(v) + "))"
      case Mul(u, v) => "(" + scalarExpr2Matlab(u) + ")*(" + scalarExpr2Matlab(v) + ")"
      case Dot(u, v) => "dot(" + vectorExpr2Matlab(u) + "," + vectorExpr2Matlab(v) + ")"
      case Par(u) => "(" + scalarExpr2Matlab(u) + ")"

      // scalar variables
      case VarScalar(s) => getVariableName(s)
      case ConstScalar(s) => getVariableName(s)
      case NumScalar(d) => "(" + d.toString + ")"
      case DeltaS(u) => "del_" + scalarExpr2Matlab(u)

      // Restricted functions/variables

      // default
      case _ => """error('MISSING SCALAR EXPR')"""
    }
  }

  def getVariableName(str_raw: String): String = {
    var s = str_raw
    val ddt = s contains "dotdot"
    val dt = s contains "dot"
    val l = s.length

    if (ddt) {
      s = s.replace("dotdot", "dd")
    }
    if (dt) {
      s = s.replace("dot", "d")
    }
    if (s contains "{") {
      s = s.replace("{", "")
    }
    if (s contains "}") {
      s = s.replace("}", "")
    }
    if (s contains "\\") {
      s = s.replace("\\", "")
    }

    return s
  }

  def generateMatlabFunction(vars: Map[String, String], filename: String): Unit = {

    // TODO: take output variables
    val FILE_PATH = new java.io.File(".").getCanonicalPath
    val directory = new File(FILE_PATH + File.separator + "output" + File.separator + "matlab")
    directory.mkdirs()
    val writer = new PrintWriter(new File(directory + File.separator + filename + ".m"))

    val date = new Date
    writer.write("function [varargout] = " + filename + "(varargin)\n")
    writer.write("%% \n")
    writer.write("% This function was generated by the Scala: Dynamics on Manifold package\n")
    writer.write(s"% $date\n")
    writer.write("% \n")
    writer.write("% NOTE: This function makes use of geometric-toolbox\n")
    writer.write("% \t\t link: https://github.com/HybridRobotics/geometric-toolbox\n")

    writer.write("%%\n\n\n")

    writer.write("%% Function Inputs\n")
    writer.write("params = varargin{1};\n\n")

    writer.write("%% \n")
    for ((key, value) <- vars) {
      writer.write(key + " = " + value + "\n\n")
    }

    writer.write("\n\n\n%% Outputs\n")
    for ((key, value) <- vars) {
      writer.write("varargout{1} = " + key + ";\n")
    }

    writer.write("\nend")
    writer.close()

  }

}
