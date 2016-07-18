// SystemGeometry.scala
package dynamics_on_manifolds

import ApplyRules._
import Differentiation._
import PrintLine._
import Variation._

object SystemGeometry
{
    
    def getXi(v:VExp) : VExp = v match
    {
        case UVec(s) => Vec(s.replace("q","xi"))
        case u:VExp => deltaV(u)
    }
    
    def getEta(m:MExp) : VExp = m match
    {
        case Mat(s) => Vec(s.replace("R","eta"))
        case m:MExp => Vec("none")
    }
    
    def getSkewEta(m:MExp) : MExp = m match
    {
        case Mat(s) => SkewMat(s.replace("R","eta"))
        case m:MExp => m
    }
    
    def getOmega(m:MExp) : VExp = m match
    {
        case Mat(s) => Vec(s.replace("R","omega"))
        case m:MExp => Vec("None")
    }
    
    def generateCollectionTerms(configVars:Tuple3[List[Exp],List[VExp],List[MExp]]) : Tuple3[List[Exp],List[VExp],List[MExp]] =
    {
        var colTerms1:List[Exp] = List()
        var colTerms2:List[VExp] = List()
        var colTerms3:List[MExp] = List()
        var scalars = configVars._1
        var vectors = configVars._2
        var matrices = configVars._3
        for (s <- scalars)
        {
            colTerms1 = colTerms1 :+ deltaS(s)
        }
        for (v <- vectors)
        {
            colTerms2 = colTerms2 :+ getXi(v)
        }
        for (m <- matrices)
        {
            colTerms2 = colTerms2 :+ getEta(m)
        }
        return Tuple3(colTerms1,colTerms2,colTerms3)
    }
    
    def generateRules(configVars:Tuple3[List[Exp],List[VExp],List[MExp]],colTerms:Tuple3[List[Exp],List[VExp],List[MExp]]) : Tuple3[Map[Exp,Exp],Map[VExp,VExp],Map[MExp,MExp]] =
    {
        var rules1:Map[Exp,Exp] = Map()
        var rules2:Map[VExp,VExp] = Map()
        var rules3:Map[MExp,MExp] = Map()
        var scalars = configVars._1
        var vectors = configVars._2
        var matrices = configVars._3

        for (v <- vectors)
        {
            if (v.isInstanceOf[UnitVector])
            {
                rules1 += (Dot(v,deltaV(v)) -> Num(0), Dot(v,diffV(v)) -> Num(0), Dot(getXi(v),v) -> Num(0))
                rules2 += (deltaV(v) -> Cross(getXi(v),v), deltaV(diffV(v)) -> VAdd(Cross(diffV(getXi(v)),v),Cross(getXi(v),diffV(v))))
            }
        }
        for (R <- matrices)
        {
            rules2 += deltaV(getOmega(R)) -> VAdd(Cross(getOmega(R),getEta(R)),diffV(getEta(R)))
            rules3 += deltaM(R) -> MMul(R,getSkewEta(R))
        }
        return Tuple3(rules1,rules2,rules3)
    }
    
    
  // list equations of motion which are functions of configuration space -> S^2, SO(3)
  def checkGeometry( configVars:Tuple3[List[Exp],List[VExp],List[MExp]], Rules:Tuple3[Map[Exp,Exp],Map[VExp,VExp],Map[MExp,MExp]] ) : Tuple3[Map[Exp,Exp],Map[VExp,VExp],Map[MExp,MExp]] = 
  {

	var g_seom:Map[Exp,Exp]   = Map()
	var g_veom:Map[VExp,VExp] = Map()
	var g_meom:Map[MExp,MExp] = Map()
  
	var vectors = configVars._2
    for (v <- vectors)
    {
		if ((applyConstraints(v dot diffV(v), Rules._1, Rules._2, Rules._3) == Num(0.0)) && (applyConstraints(v dot diffV(v), Rules._1, Rules._2, Rules._3) == Num(0.0)))
		{
			g_veom += (diffV(v) -> Cross(Vec("omega" + printVTree(v).replaceFirst("q","")),v))
		}
	}	
	
	var matrices = configVars._3
    for (m <- matrices)
    {
        g_meom += (MMul(m,SkewMat("omega" + printMTree(m).replaceFirst("R",""))) -> diffM(m))
	}
	
	var geometrical_eoms = Tuple3(g_seom, g_veom, g_meom)
	
    return geometrical_eoms
  }
}