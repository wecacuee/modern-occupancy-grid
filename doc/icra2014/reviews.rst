##########################
Points Raised by Reviewers
##########################


[X] Are thicker walls really better?
====================================

-Walls in maps are shown to be thicker than in the ground
truth datasets. It is clear from the Freiburg dataset that
a tendency for modeling thin walls can lead to holes (e.g.
free space cells that should be occupied), but on the other
hand, generating thicker walls also can lead to regions
becoming impassable in a practical (robot navigation)
setting. It thus is not clear that thicker walls always
have to be an improvement.

Rebuttal: Nothing to do
~~~~~~~~~~~~~~~~~~~~~~~

Qualitative results do not show this improvement. But lower energy is
a quantitative measure of the map being closer to the desired objective
function.


[X] Show experiments on applications like navigation
=====================================================

-A very convincing demonstration of the improvements
obtained by use of the proposed techniques would be the use
maps in a application scenario (for example robot
navigation) and demonstrating that performance in this
scenario is improved as compared to the previous state of
the art.


-It would be a very interesting further direction of
research to evaluate potential improvements to SLAM
approaches (i.e. using the proposed system online) by using
the presented techniques.

Rebuttal : TODO Journal
~~~~~~~~~~~~~~~~~~~~~~~

Good ideas for journal version of the paper.


[X] Corrections about Figure 4
==============================

Some remarks regarding Fig. 4:
-The upper ground truth map seems to be wrong. Walls
existing in that map are not visible in the ground truth
information, instead there is a direct transition from free
to unkown in all islands in the ground truth map.
-It is not clear why unknown space is not colored grey in
the lowest row of the illustration for most of the
presented approaches.

Rebuttal : Addressed
~~~~~~~~~~~~~~~~~~~~~~~~~~

The resizing of original floorplan was causing walls to be invisible in the
resized ground truth. Fixed that.

[X] How about performance on RGBD sensor and sonar?
===================================================

However, I believe that the authors could have provided a
deeper
evaluation.  In particular, the use of only laser sensor
seems to me
rather restrictive to show the power of the proposed
method. I would e
curious to see the proposed approaches in action on more
noisy data
e.g. RGB-D or sonar.  In addition to that, I think the
authors should
refer the work of Ruhnke et. al, which proposed some sort
of
surface-based bundle adjustment to refine the quality of 2D
maps. Whereas this does not fall in the same exact category
of the
proposed approach, I think it is worth referencing, and
potentially
comparing to it.

Rebuttal : TODO Journal
~~~~~~~~~~~~~~~~~~~~~~~

Good idea for journal version of the paper.


[X] No experiments on computational efficiency
==============================================

Also the authors have put efforts into the computational
efficiency of the approaches. But there seems to be no
result supporting it! As occupancy grid mapping is quite
expensive considering simulations can also aggregate a lot
of sensor measurements it would be nice to supplement the
final version of the paper supporting the computational
efficiency.	

Rebuttal: Nothing to do
~~~~~~~~~~~~~~~~~~~~~~~

The results shown in Fig. 3 are about computational efficiency.
The inverse sensor model is starting point of all the attempted
algorithms.

[X] Compare with gmapping
=========================

1. In general, the problem of grid mapping is considered a
solved problem. There are several standard algorithms that
do the job well, e.g. the Gmapping algorithm. Why did the
authors choose to re-solve this problem with a theory that
seems rather complex. On this point, this paper provides
the impression that an old problem is studied again using a
much more complex new technique.

Rebuttal: Nothing to do
~~~~~~~~~~~~~~~~~~~~~~~

Gmapping uses inverse sensor model.

[X] More datasets
=================

[X] Do we really need complex techniques
========================================

2. The experimental results shown in this paper are
insufficient. First of all, the authors show the
performance of their algorithm only on three small data
sets. For such an old problem, namely grid mapping, more
results and results for big maps are expected. The
quantitative results need also be extended, at least for
the other two data sets. (Currently only for one data set).
In addition, in the qualitative results shown in Figure 4,
no improvements for the first two row can be found, if we
compare the subfigures shown in column no. 2 with the ones
in column no. 5 and 6. This leads back to the above
question, why do we need this new technique for solving the
problem of grid mapping?

Rebuttal: TODO Journal
~~~~~~~~~~~~~~~~~~~~~~~

Interesting questions. Should be addressed in the journal version.

####################
CommentsToAuthor.txt
####################

The paper is well-written and easy to read. I enjoyed
reading the paper and the good reviews about it. The
validation in the experiments could be improved.

----------------------------------------
Comments on Video Attachment:

####################
Full text of reviews
####################

#############
Review645.txt
#############

Reviewer 2 of ICRA 2014 submission 1400

Comments to the author
======================

In the paper novel approaches for occupancy grid mapping
using recent inference techniques are presented.  In the
introduction section, the established approaches of mapping
using inverse and forward sensor models are described, as
well as the used inference methods. After the problem
definition section providing more details, the MCMC based
Metropolis Hastings algorithm as well as different MAP
estimation approaches and their application to the
occupancy grid mapping problem are explained
comprehensively. Experiments using both simulated and real
world data are used to demonstrate the application and
performance of the newly developed methods. The inference
based methods are shown to have better performance than the
sampling based and traditional inverse sensor model based
methods.
The paper is written in a technically very sound and well
understandable way. While the presented MAP techniques are
not new, their application to occupancy grid mapping is
novel and shown to provide significant improvements over
the established state of the art. There are some comments
that can be made about the evaluation section:
-Walls in maps are shown to be thicker than in the ground
truth datasets. It is clear from the Freiburg dataset that
a tendency for modeling thin walls can lead to holes (e.g.
free space cells that should be occupied), but on the other
hand, generating thicker walls also can lead to regions
becoming impassable in a practical (robot navigation)
setting. It thus is not clear that thicker walls always
have to be an improvement.
-A very convincing demonstration of the improvements
obtained by use of the proposed techniques would be the use
maps in a application scenario (for example robot
navigation) and demonstrating that performance in this
scenario is improved as compared to the previous state of
the art.
-It would be a very interesting further direction of
research to evaluate potential improvements to SLAM
approaches (i.e. using the proposed system online) by using
the presented techniques.

Some remarks regarding Fig. 4:
-The upper ground truth map seems to be wrong. Walls
existing in that map are not visible in the ground truth
information, instead there is a direct transition from free
to unkown in all islands in the ground truth map.
-It is not clear why unknown space is not colored grey in
the lowest row of the illustration for most of the
presented approaches.

#############
Review647.txt
#############

Reviewer 3 of ICRA 2014 submission 1400

Comments to the author
======================

This paper proposes a detail analysis of Occupancy Grid
mapping using
forward sensors models by using a factor-graph formalism. 
To the best
of my knowledge, the authors are the first casting
Occupancy Mapping
as a Factor-graph optimization.  In this domain, the
authors
investigate a family of increasingly complex and
domain-aware
inference approaches to address the mapping problem.

From the scientific point of view, I regard this work as
complete and
rich of insights.  The techniques and the structures used
are
explained in great detail.

However, I believe that the authors could have provided a
deeper
evaluation.  In particular, the use of only laser sensor
seems to me
rather restrictive to show the power of the proposed
method. I would e
curious to see the proposed approaches in action on more
noisy data
e.g. RGB-D or sonar.  In addition to that, I think the
authors should
refer the work of Ruhnke et. al, which proposed some sort
of
surface-based bundle adjustment to refine the quality of 2D
maps. Whereas this does not fall in the same exact category
of the
proposed approach, I think it is worth referencing, and
potentially
comparing to it.

Overall this work is scientifically valid, and provides
useful
insights and effective formalisms. The experimental
validation can be
significantly improved.


[References]
[1] Highly Accurate Maximum Likelihood Laser Mapping by
Jointly Optimizing Laser Points and Robot Poses by M.
Ruhnke, R. Kümmerle, G. Grisetti, W. Burgard. In: Proc. of
the IEEE Int. Conf. on Robotics & Automation (ICRA), 2011

#############
Review863.txt
#############

Reviewer 4 of ICRA 2014 submission 1400

Comments to the author
======================

The work presented in this paper approaches the problem of
occupancy grid mapping, a large non-linear problem, using
Factor Graphs. The authors have introduced the Factor Graph
formalization and then applied inference algorithms in the
form of Belief Propagation and Dual Decomposition. 

The paper starts with weighing the use of forward sensor
model in place of the inverse sensor model and thus
re-formalizing the problem excluding the independence of
the occupancy of grid cells on each other. Then a possible
representation is given for the reduced problem in terms of
bipartite Factor Graphs. As some other reference work have
addressed the authors then adapt a general Markov Chain
Monte Carlo method, Metropolis Hastings algorithm, to the
factor graph representation. Later they show in the
qualitative and quantitative analysis that this approach is
bound to get stuck in local minima and thus not optimum.
Although important information which is missing is the
dependence on the 
number of iterations used which when comparing with the
main contributions of the paper should be mentioned
clearly. Also as metopolis hastings' algo assumes
dependence on consecutive steps/iterations/selections it
would be interesting to see how it performs when an
independent transition probability is chosen in place of
symmetric.  

The next sections apply the Sum Product Algorithm and Dual
Decomposition on the MAP problem using the Factor Graph
formalization. The same algorithms are stated in an
efficient way and experimental results are shown for
piecewise constant sensor model. The results are quite
decent both quantitatively and qualitatively but 
they
are compared with just the inverse sensor model and the
MCMC algorithm from previous section. It would be nice to
see a comparison with other methods using the forward
sensor model also.

Also the authors have put efforts into the computational
efficiency of the approaches. But there seems to be no
result supporting it! As occupancy grid mapping is quite
expensive considering simulations can also aggregate a lot
of sensor measurements it would be nice to supplement the
final version of the paper supporting the computational
efficiency.	

##############
Review9109.txt
##############

Reviewer 5 of ICRA 2014 submission 1400

Comments to the author
======================

In this paper the authors propose to solve the problem of
grid mapping using higher order factor graphs and modern
MAP inference methods. This paper shows an original concept
which is theoretically sound. The point of view that the
dependencies between the grid cells need be taken into
account is also quite interesting. In general, the reviewer
considers this paper a candidate for acceptance for
ICRA2014, however, the following problems definitely
lowered the quality of this paper:
1. In general, the problem of grid mapping is considered a
solved problem. There are several standard algorithms that
do the job well, e.g. the Gmapping algorithm. Why did the
authors choose to re-solve this problem with a theory that
seems rather complex. On this point, this paper provides
the impression that an old problem is studied again using a
much more complex new technique.

2. The experimental results shown in this paper are
insufficient. First of all, the authors show the
performance of their algorithm only on three small data
sets. For such an old problem, namely grid mapping, more
results and results for big maps are expected. The
quantitative results need also be extended, at least for
the other two data sets. (Currently only for one data set).
In addition, in the qualitative results shown in Figure 4,
no improvements for the first two row can be found, if we
compare the subfigures shown in column no. 2 with the ones
in column no. 5 and 6. This leads back to the above
question, why do we need this new technique for solving the
problem of grid mapping?

Taking all the above aspects into account, the reviewer
classifies this paper into the category of high borderline.
