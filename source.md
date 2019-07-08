<section id="themes">
	<h2>Themes</h2>
		<p>
			Set your presentation theme: <br>
			<!-- Hacks to swap themes after the page has loaded. Not flexible and only intended for the reveal.js demo deck. -->
                        <a href="#" onclick="document.getElementById('theme').setAttribute('href','css/theme/black.css'); return false;">Black (default)</a> -
			<a href="#" onclick="document.getElementById('theme').setAttribute('href','css/theme/white.css'); return false;">White</a> -
			<a href="#" onclick="document.getElementById('theme').setAttribute('href','css/theme/league.css'); return false;">League</a> -
			<a href="#" onclick="document.getElementById('theme').setAttribute('href','css/theme/sky.css'); return false;">Sky</a> -
			<a href="#" onclick="document.getElementById('theme').setAttribute('href','css/theme/beige.css'); return false;">Beige</a> -
			<a href="#" onclick="document.getElementById('theme').setAttribute('href','css/theme/simple.css'); return false;">Simple</a> <br>
			<a href="#" onclick="document.getElementById('theme').setAttribute('href','css/theme/serif.css'); return false;">Serif</a> -
			<a href="#" onclick="document.getElementById('theme').setAttribute('href','css/theme/blood.css'); return false;">Blood</a> -
			<a href="#" onclick="document.getElementById('theme').setAttribute('href','css/theme/night.css'); return false;">Night</a> -
			<a href="#" onclick="document.getElementById('theme').setAttribute('href','css/theme/moon.css'); return false;">Moon</a> -
			<a href="#" onclick="document.getElementById('theme').setAttribute('href','css/theme/solarized.css'); return false;">Solarized</a>
		</p>
</section>

H:

# Kinematics on nub

Sebastian Chaparro

H:

# Index

 1. Introduction<!-- .element: class="fragment" data-fragment-index="1"-->
 2. Forward Kinematics<!-- .element: class="fragment" data-fragment-index="2"-->
 3. Inverse Kinematics <!-- .element: class="fragment" data-fragment-index="3"-->
 4. IK Heuristic Methods <!-- .element: class="fragment" data-fragment-index="4"-->
 5. Using constraints <!-- .element: class="fragment" data-fragment-index="5"-->
 6. Demos <!-- .element: class="fragment" data-fragment-index="6"-->

H:

## Introduction


 * How to bring to life animated objects? <!-- .element: class="fragment" data-fragment-index="1"-->

 * One approach is [skeletal animation](https://en.wikipedia.org/wiki/Skeletal_animation) where the object to animate is represented by a skeleton and a skin.
 <!-- .element: class="fragment" data-fragment-index="2"-->  


V:

## Skeleton

* Set of rigid multibody system called bones (or links) attached by joints. A skeleton usually is represented as a hierarchical structure. <!-- .element: class="fragment" data-fragment-index="1"--> 
* Joints are parametrized by Degrees of Fredom (DOF). <!-- .element: class="fragment" data-fragment-index="2"--> 

* An end effector $\mathbf{s}$ is a point of interest that depends on joint configurations: $ \mathbf{s} = f(\mathbf{ \theta }) $ <!-- .element: class="fragment" data-fragment-index="3"-->
* <!-- .element: class="fragment" data-fragment-index="4"--> Skeleton is used to define or modify movements (e.g. [Keyframe animation](https://www.utdallas.edu/atec/midori/Handouts/keyframing.htm), [Procedural animation](https://www.alanzucconi.com/2017/04/17/procedural-animations/), [MoCap data](https://en.wikipedia.org/wiki/Motion_capture)). 



V:

<figure class="fragment" data-fragment-index="4">
    <img data-src='fig/fig1.jpg'/>
</figure>


V:

## Skin

* Once motion is set, it is required to bind the skeleton with a surface (2D or 3D mesh).  
<!-- .element: class="fragment" data-fragment-index="1"-->

* Skeleton motion must deform the mesh smoothly (e.g when we bend the elbow, the skin around the bones stretches and shrinks).
<!-- .element: class="fragment" data-fragment-index="2"-->

* To do so, given a mesh vertex it is defined an influence weight per skeleton joint. The vertex is deformed according to joint transformations. 
<!-- .element: class="fragment" data-fragment-index="3"-->

* Check this short example [video](https://youtu.be/YXDzMZaAo0U). 
<!-- .element: class="fragment" data-fragment-index="4"-->

V:

## Skinning
<iframe width="100%" height="500px" data-src="videos/Skinning.webm"></iframe>

H:

## Forward Kinematics (FK)
<section>
  <div style="text-align: justify-all; float: left; width: 50%">
  <br>
  <br>
    <ul style="text-align: justify-all; font-size: 1em !important;">
      <li class="fragment" data-fragment-index="1"> Given the joint configurations $ \mathbf{ \theta }$ find the End effector Position $ \mathbf{s} = f(\mathbf{ \theta })$ </li>
      <br>
      <li class="fragment" data-fragment-index="2"> Direct joint manipulation </li>
      <br>
      <li class="fragment" data-fragment-index="3"> Exhaustive </li>
      <br>
      <li class="fragment" data-fragment-index="4"> Not Redundant </li>
      <br>
    </ul>
  </div>
  <div style="text-align: justify-all; float : left; width : 50%" class=embed-container >
    <br>
    <br>
    <iframe class="fragment" data-fragment-index="5" width="100%" height="500px" data-src="videos/FK.webm"></iframe>
  </div>
</section>

H:

## Inverse Kinematics (IK)
  <div style="text-align: justify-all; float: left; width: 50%">
    <ul style="text-align: justify-all; font-size: 1em !important;">
      <li class="fragment" data-fragment-index="1"> Given the state of the Final effector $ \mathbf{s} $ find joint configurations: $\mathbf{ \theta } = f^{-1}( \mathbf{ s}) $ </li>
      <br>
      <li class="fragment" data-fragment-index="2"> Indirect joint manipulation based on Goal Reaching </li>
      <br>
      <li class="fragment" data-fragment-index="3"> Root joint is Fixed </li>
      <br>
      <li class="fragment" data-fragment-index="4"> Not Exhaustive </li>
      <br>
      <li class="fragment" data-fragment-index="5"> Redundant </li>
      <br>
    </ul>
    <!-- more Elements -->
  </div>
  <div style="text-align: justify-all; float : left; width : 50%" class=embed-container >
    <br>
    <iframe class="fragment" data-fragment-index="6" width="100%" height="500px" data-src="videos/IK.webm"></iframe>
  </div>

V:

## Requirements

IK on interactive applications must be:
* R1 Efficient: Take as little time as possible.
<!-- .element: class="fragment" data-fragment-index="1"-->
* R2 Accurate: Reach the goal position / orientation.
<!-- .element: class="fragment" data-fragment-index="2"-->
* R3 Scalable: Work with Big amounts of DOF. 
<!-- .element: class="fragment" data-fragment-index="3"-->
* R4 Robust: Reach the goal when managing constraints.
<!-- .element: class="fragment" data-fragment-index="4"-->
* R5 Able to Generate natural poses.
<!-- .element: class="fragment" data-fragment-index="5"-->
* R6 Generic: Deal with arbitrary Figures.
<!-- .element: class="fragment" data-fragment-index="6"-->
V:
## IK Methods
| Kind      | R1 | R2 | R3 | R4 | R5 | R6 |
|-----------|-----------|----------|----------|-------------|---------|---------|
| Analitycal|     X     |     X    |    -     |      X      |    X    |    -    |
| Numerical |     -     |     X    |    X     |      X      |    -    |    X    |
| ** Numerical Heuristic (FABRIK) ** |     X     |     X    |    X     |      -      |    X    |    X    |

H:
# IK Heuristic Methods

V: 
## Cyclic Coordinate Descent (CCD)
Proposed by [Wang and Chen on 1991](http://web.cse.ohio-state.edu/~parent.1/classes/788/Sp06/ReferenceMaterial/IK/WC91.pdf)
<div style="text-align: justify-all;">
<br>
  <ul style="text-align: justify-all; font-size: 1em !important;">
    <li class="fragment" data-fragment-index="1"> Works only on Kinematic chains. </li>
    <li class="fragment" data-fragment-index="2"> Let $ \mathbf{v\_{ie}} $ the vector formed by the $ith$ joint and the end effector position (Yellow one). </li>
    <li class="fragment" data-fragment-index="3"> Let $ \mathbf{v\_{it}} $ the vector formed by the $ith$ joint and the target position (Green one). </li>
    <li class="fragment" data-fragment-index="4"> Modify each Joint configuration per iteration to reduce the error: </li>
  </ul>
    <div class="fragment" data-fragment-index="5">
    $$ cos(\theta \_{i}) = \frac{ \mathbf{v\_{ it }} } { \left| \mathbf{v \_{ it }} \right| } \frac{ \mathbf{v \_{ie}} }{ \left| \mathbf{v\_ {ie}} \right| } , \mathbf{r} = \mathbf{v\_{ it }} \times \mathbf{v\_{ ie }}$$ 
    </div>

</div>

V:
## Cyclic Coordinate Descent (CCD)
<iframe width="100%" height="500px" data-src="videos/CCD_Solver_1.webm"></iframe>

V: 
## Forward and Backward Reaching Inverse Kinematics (FABRIK)
Proposed by [Andreas Aristidou on 2009](http://www.andreasaristidou.com/publications/papers/FABRIK.pdf)
<div style="text-align: justify-all;">
  <ul style="text-align: justify-all; font-size: 0.9em !important;">
    <li class="fragment" data-fragment-index="1"> <i>"Minimize error by adjusting each joint angle one at a time". </i></li>
    <li class="fragment" data-fragment-index="2"> Let $ \mathbf{p}\_i$ the position of the $ ith $ joint in a chain, with $ i \in \\{ 1,2,...,n \\}$, $\mathbf{p}\_1$ the root of the chain, $\mathbf{p}\_n$ the end effector and $\mathbf{t}$ the target position. </li>
    <li class="fragment" data-fragment-index="3"> Move the structure while keeping distances  $ d\_i = \left| \mathbf{p}\_i - \mathbf{p}\_{i+1} \right| $ between Joints (bones are rigid) via finding a point on a line. </li>
    <li class="fragment" data-fragment-index="4"> A full iteration is composed of two stages: 
    <ul style="text-align: justify-all; font-size: 0.8em !important;">
      <li class="fragment" data-fragment-index="5"> <b>Foward stage</b>: Assume that the target $\mathbf{t}$ is reached by end effector $\mathbf{p}\_n$ and adjust the distances of the remaining Joints. </li>
      <li class="fragment" data-fragment-index="6"> <b>On Backward stage</b>: move the root $\mathbf{p}\_1$ to its initial position and adjust the distances of the remaining Joints. </li>
    </li>
    </ul>

  </ul>
</div>

V:
## FABRIK
<iframe width="100%" height="500px" data-src="videos/FABRIK_Solver_1.webm"></iframe>

H:
## Using constraints
  <div style="text-align: justify-all; float: left; width: 50%">
  <br>
    <ul style="text-align: justify-all; font-size: 1em !important;">
      <li class="fragment" data-fragment-index="1"> When end effectors are manipulated we expect to obtain intuitive poses.</li>
      <br>
      <li class="fragment" data-fragment-index="2"> There could exist many solutions (i.e many different poses) that satisfy the IK problem. </li>
      <br>
      <li class="fragment" data-fragment-index="3"> Limiting the movement of the skeleton could enhance IK performance. </li>
    </ul>
  </div>
  <div style="text-align: justify-all; float : left; width : 50%" class=embed-container >
    <br>
    <iframe class="fragment" data-fragment-index="4" width="100%" height="500px" data-src="videos/multiple_solutions.webm"></iframe>
  </div>

V:
## Using constraints
  <div style="float: left; width: 50%" >
  <p class="fragment" data-fragment-index="1" style="text-align: left; font-size: 0.9em !important;">
  Limiting the joint movement locally by enclosing its related segment on a volume.
  </p>
  <ul style=" padding-left:40px; text-align: left; font-size: 0.7em !important;">
    <li class="fragment" data-fragment-index="1">[Fast and Easy Reach-Cone Joint Limits](https://pdfs.semanticscholar.org/d535/e562effd08694821ea6a8a5769fe10ffb5b6.pdf)</li>
    <li class="fragment" data-fragment-index="1">[A joint-constraint model using signed distance fields](https://link.springer.com/article/10.1007/s11044-011-9296-1)</li>
  </ul>
  <p class="fragment" data-fragment-index="2" style="text-align: left; font-size: 0.9em !important;">
  Using physical attributes.
  </p>
  <ul class="fragment" data-fragment-index="2" style=" padding-left:40px; text-align: left; font-size: 0.7em !important;">
    <li> [An Efficient Energy Transfer Inverse Kinematics Solution](https://pdfs.semanticscholar.org/aac6/cbd168f0e01911edbe564f59d7c1a00b7535.pdf)</li>
  </ul>
  <p class="fragment" data-fragment-index="3" style="text-align: left; font-size: 0.9em !important;">
  Locking a joint position or orientation.
  </p>
  <ul class="fragment" data-fragment-index="3" style=" padding-left:40px; text-align: left; font-size: 0.7em !important;">
    <li> [Nailing and pinning: Adding constraints to inverse kinematics](https://otik.uk.zcu.cz/bitstream/11025/11239/1/Greeff.pdf)</li>
  </ul>
  </div>
  <div style="text-align: middle; float = right; width = 50%; height = 100%">
    <div style="text-align: middle; float: right;">
      <figure class="fragment" data-fragment-index="1">
          <img width = 80% data-src='fig/fig7.png'/>
      </figure>
      <figure class="fragment" data-fragment-index="3">
          <img width = 80% data-src='fig/fig9.png'/>
      </figure>
      <!-- more Elements -->
    </div>
  </div>

V:
## Using local constraints - Example
  <div style="text-align: justify-all; float: left; width: 50%">
  <br>
    <ul style="text-align: justify-all; font-size: 1em !important;">
      <li class="fragment" data-fragment-index="1"> Assume that Node 0 rotate only around a fixed axis (1 DOF).</li>
      <br>
      <li class="fragment" data-fragment-index="2"> Assume that Node 0 rotation is enclosed by a minimum and a maximum angle. </li>
      <br>
      <li class="fragment" data-fragment-index="3"> With local constraints there's a unique solution when target is reachable. </li>
    </ul>
  </div>
  <div style="text-align: justify-all; float : left; width : 50%" class=embed-container >
    <br>
    <iframe class="fragment" data-fragment-index="5" width="100%" height="500px" data-src="videos/constraint_1.webm"></iframe>
  </div>

V: 
## Hinge constraint
1-DOF rotational constraint. i.e the node will rotate only around a single direction. Furthermore, the rotation made by the constrained node is enclosed on a minimum and maximum angle.
<iframe width="100%" height="400px" data-src="videos/hinge_interactive.webm"></iframe>

V: 
## Ball and socket constraint
3-DOF rotational constraint (the node could rotate around any direction) that decomposes a rotation into two components called Swing (2-DOF) and Twist (1-DOF) rotations and limits each of them (see [FABRIK paper](http://www.andreasaristidou.com/publications/papers/FABRIK.pdf)).
<iframe width="100%" height="400px" data-src="videos/BallAndSocket.webm"></iframe>


H:

# DEMOS

V:
## Getting skeleton from Data
There are different kind of files as [Collada](https://www.khronos.org/collada/) or [BVH](https://research.cs.wisc.edu/graphics/Courses/cs-838-1999/Jeff/BVH.html) to "transport 3D assets between applications". Here we are interested on skeletal structure.
<div style="text-align: justify-all; float: left; width: 50%" class=embed-container >
  BVH Demo
  <iframe width="100%" height="400px" data-src="videos/bvh_example.webm"></iframe>
</div>
<div style="text-align: justify-all; float : left; width : 50%" class=embed-container >
  Collada Demo
  <iframe width="100%" height="400px" data-src="videos/dae_example.webm"></iframe>
</div>

V:
## Building & Interacting
Allow the user to define and interact with the skeletal structure easyly.
<div style="text-align: justify-all; float: left; width: 50%" class=embed-container >
  Saying HI!
  <iframe width="100%" height="500px" data-src="videos/demo_2.webm"></iframe>
</div>
<div style="text-align: justify-all; float : left; width : 50%" class=embed-container >
  Multiple end Effectors
  <iframe width="100%" height="500px" data-src="videos/demo_skeleton.webm"></iframe>
</div>

V:
## Skinning
Allow the user to define a skeleton and bind it to a mesh.
<iframe width="100%" height="500px" data-src="videos/builder_demo_high_speed.webm"></iframe>

V:
## Procedural Animation
<div style="text-align: justify-all; float: left; width: 50%" class=embed-container >
  Fish Demo
  <iframe width="100%" height="500px" data-src="videos/fish_demo.webm"></iframe>
</div>
<div style="text-align: justify-all; float : left; width : 50%" class=embed-container >
  Flock Demo
  <iframe width="100%" height="500px" data-src="videos/flock_demo.webm"></iframe>
</div>

V:
## Procedural Animation
Eagle Demo
<iframe width="100%" height="500px" data-src="videos/eagle_demo.webm"></iframe>

V:
## Procedural Animation
Multilegged gait simulation
<div style="text-align: justify-all;" class=embed-container >
	<iframe width="100%" height="500px" data-src="videos/procedural_demo.webm"></iframe>
</div>

H:

## References

* [Introduction to IK](https://www.math.ucsd.edu/~sbuss/ResearchWeb/ikmethods/index.html)

* [Inverse Kinematics Techniques in Computer Graphics: A Survey](http://www.andreasaristidou.com/InverseKinematics.html)

* [Forward and Backward Reaching Inverse Kinematics (FABRIK)](http://www.andreasaristidou.com/publications.html)

* [Cyclic Coordinate Descent (CCD)](https://sites.google.com/site/auraliusproject/ccd-algorithm)
