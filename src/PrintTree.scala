// PrintTree.scala
package dynamics_on_manifolds

import Collection._
import Expansion._
import Separation._
import Simplification._

object PrintLine
{

  // Print Matrix to Screen
  def printMTree( m:MExp ) : String = m match
  {
    case SMMul(a,b) => printMTree(a) + "*" + printTree(b) 
    case MAdd(a,b)  => "(" + printMTree(a) + "+" + printMTree(b) + ")" 
    case Mat(s)   => s
    case SMat(s)  => s
    case CSMat(s)  => s
    case CMat(s)  => s
    case SkewMat(s)  => s
    case MMul(s,r)  => printMTree(s) + "*" + printMTree(r)
    case deltaM(s)  => "deltaV(" + printMTree(s) + ")"
    case transpose(s)  => "transpose(" + printMTree(s) + ")" 
  }

  // Print Vector to Screen
  def printVTree( v:VExp ) : String = v match
  {
    case deltaV(u) => "delta" +"(" + printVTree(u) + ")"
    case Vec(s) => s
    case UVec(s) => s
    case CVec(s) => s
    case VAdd(u,v) => printVTree(u) + "+" + printVTree(v)
    case Cross(u,v) => printVTree(u) + "x" + printVTree(v)
    case SMul(u,v) => printVTree(u) + "*" + printTree(v) 
    case VMul(u,v) => printMTree(u) + "*" + printVTree(v) 
	case ZVec(s) => s
  }

  // Print Scalar to Screen
  def printTree( e:Exp ) : String = e match
  {
    case deltaS(u) => "delta" +"(" + printTree(u) + ")"
    case Var(s) => s
    case Cons(s) => s
    case Num(d) => d.toString
    case Add(u,v) => printTree(u) + "+" + printTree(v)
    case Mul(deltaS(u),v) => printTree(deltaS(u)) + "* (" + printTree(v) + ")"
    case Mul(u,v) => printTree(u) + "*" + printTree(v)
    case Dot(u,v) => printVTree(u) + "." + "(" + printVTree(v) + ")"
    case Par(u)   => "(" + printTree(u) + ")" 
  }


  // Prints set of Equations of Motion
  def printEOM( e:Exp, s:List[Exp], v:List[VExp] ) : Tuple3[String,List[Exp],List[VExp]] =
  {
    var eom = "Equations of Motion:"
    var piece:Exp = Num(0)
    var counter:Int = 0
	var eom_s:List[Exp] = List()
	var eom_v:List[VExp] = List()
    
    // Extracts one equation for each collection term
    for ( scalar <- s)
    {
        piece = colScalar(e,scalar)
        piece = separate(piece, scalar)
        val line = simplify(piece)
        val finalOutput = removeOneCoefs(line)
        eom = eom + "\nLag" + (counter+1).toString + ": " + printTree(finalOutput) + " = 0"
        eom_s = eom_s :+ finalOutput
        counter = counter + 1
    }
    
    // Extracts one equation for each collection term
    for ( vector <- v)
    {
        piece = col(e,vector)
        piece = separateV(piece, vector)
        val prep = simplify(piece)
        
        val line = extractAll(prep)
        
        val finalOutput = removeOneCoefsV(line)
        eom = eom + "\nLag" + (counter+1).toString + ": " + printVTree(finalOutput) + " = 0"
        eom_v = eom_v :+ finalOutput
        counter = counter + 1
    }
    return Tuple3(eom,eom_s,eom_v)
  }

}