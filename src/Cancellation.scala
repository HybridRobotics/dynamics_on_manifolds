// CancelTerms.scala
package dynamics_on_manifolds


import Separation._
import Simplification._

object Cancellation
{

  // Get rid of zero-terms
  def split( e:Exp ) : Tuple2[List[Exp],List[Exp]] =
  {
    var ordTerms:List[Exp] = List()
    var numConsts:List[Exp] = List()
    var sep:Tuple2[Exp,Exp] = (Num(0),e) 
    var constSep:Tuple2[Exp,Exp] = (Num(0),Num(0))

    while (sep._2 != Num(0))
    {

        sep = splitter(sep._2)
        constSep = numMultiple(sep._1)
        ordTerms = ordTerms :+ constSep._2
        numConsts = numConsts :+ constSep._1
    }
    var split:Tuple2[List[Exp],List[Exp]] = Tuple2(numConsts,ordTerms)
    return split
  }

  // identify like terms
  def cancelTerms( consts:List[Exp], terms:List[Exp] ) : Exp =
  {
    var simpTerms:List[Exp] = List()
    var nc = consts
    for ( i  <- 0 to (terms.length-1))
    {
      for (j <- i+1 to (terms.length-1))
      {
        if (terms(i) == terms(j))
        {
          nc = nc.updated(j,simplify(nc(i)+nc(j)))
          nc = nc.updated(i,Num(0))
        }
      }
    }
    for ( i  <- 0 to (terms.length-1))
    {
      simpTerms = simpTerms :+ Mul(nc(i),terms(i))
    }
    return sumTerms(simpTerms)
  }

  // add numberic constant to like terms
  def sumTerms( terms:List[Exp] ) : Exp =
  {
        var total:Exp = Num(0)
        for ( term <- terms ) {total = term + total}
        return total
  }

}