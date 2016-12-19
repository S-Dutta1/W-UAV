/*
* Copyright (c) 2006-2009 Erin Catto http://www.box2d.org
*
* This software is provided 'as-is', without any express or implied
* warranty.  In no event will the authors be held liable for any damages
* arising from the use of this software.
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely, subject to the following restrictions:
* 1. The origin of this software must not be misrepresented; you must not
* claim that you wrote the original software. If you use this software
* in a product, an acknowledgment in the product documentation would be
* appreciated but is not required.
* 2. Altered source versions must be plainly marked as such, and must not be
* misrepresented as being the original software.
* 3. This notice may not be removed or altered from any source distribution.
*/

/* 
 * Base code for CS 251 Software Systems Lab 
 * Department of Computer Science and Engineering, IIT Bombay
 * 
 */


#include "cs251_base.hpp"
#include "render.hpp"

#ifdef __APPLE__
	#include <GLUT/glut.h>
#else
	#include "GL/freeglut.h"
#endif

#include <cstring>
using namespace std;

#include "dominos.hpp"

namespace cs251
{
  /**  The is the constructor 
   * This is the documentation block for the constructor.
   */ 
  
  dominos_t::dominos_t()
  {
    //Ground
    /*! \var b1 
     * \brief pointer to the body ground 
     */ 
    b2Body* b1;  
    {
      
      b2EdgeShape shape; 
      shape.Set(b2Vec2(-90.0f, 0.0f), b2Vec2(90.0f, 0.0f));
      b2BodyDef bd; 
      b1 = m_world->CreateBody(&bd); 
      b1->CreateFixture(&shape, 0.0f);
    }

     // ////////  random fires   //////////////
    
     //  b2Body* ;b2Body* blimp1;
      
     //  b2CircleShape circle;
     //  circle.m_radius = 5.0;
  
     //  b2FixtureDef bmp;
     //  bmp.shape = &circle;
     //  bmp.density = 50.0f;
     //  bmp.friction = 0.0f;
     //  bmp.restitution = 0.0f;
     //  b2BodyDef ballbd;
     //  ballbd.type = b2_dynamicBody;


     //  b2FixtureDef bmp1;
     //  bmp1.shape = &circle;
     //  bmp1.density = 50.0f;
     //  bmp1.friction = 0.0f;
     //  bmp1.restitution = 0.0f;
     //  b2BodyDef ballbd1;
     //  ballbd1.type = b2_dynamicBody;

     //  bmp.filter.categoryBits = 0x0001;
     //  bmp1.filter.categoryBits = 0x0002;
     //  bmp.filter.maskBits = 0x0000;
     //  bmp1.filter.maskBits = 0x0000;

     //  ballbd.position.Set(0.0f, 10.0f);
     //  blimp = m_world->CreateBody(&ballbd);
     //  blimp->CreateFixture(&bmp);

     //  ballbd1.position.Set(0.0f, 30.0f);
     //  blimp1 = m_world->CreateBody(&ballbd1);
     //  blimp1->CreateFixture(&bmp1);
       
      

  
      
    }     
    
  

  sim_t *sim = new sim_t("Dominos", dominos_t::create);
}
