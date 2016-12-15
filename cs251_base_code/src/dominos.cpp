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

    //  ////circular blimp///////////
    // {
    //   b2Body* blimp;
      
    //   b2CircleShape circle;
    //   circle.m_radius = 5.0;
  
    //   b2FixtureDef bmp;
    //   bmp.shape = &circle;
    //   bmp.density = 50.0f;
    //   bmp.friction = 0.0f;
    //   bmp.restitution = 0.0f;
    //   b2BodyDef ballbd;
    //   ballbd.type = b2_staticBody;
    //   ballbd.position.Set(-30.0f, 30.0f);
    //   blimp = m_world->CreateBody(&ballbd);
    //   blimp->CreateFixture(&bmp);
    // }    


    ///////ellipse blimp/////////////////////
    {
      b2Body* tblp;

      b2Vec2 vertices[20];
      vertices[0].Set(0.5f, 0);

      vertices[1].Set(2, 2);
      vertices[2].Set(5, 2);
      vertices[3].Set(8, 4);
      vertices[4].Set(10,5);
      vertices[5].Set(10,6);
      vertices[6].Set(8,7);
      vertices[7].Set(5,10);
      vertices[8].Set(2,10);

      vertices[9].Set(0.5f,12);
      vertices[10].Set(-0.5f,12);

      vertices[11].Set(-2, 10);
      vertices[12].Set(-5, 10);
      vertices[13].Set(-8, 7);
      vertices[14].Set(-10, 6);
      vertices[15].Set(-10,5);
      vertices[16].Set(-8,4);
      vertices[17].Set(-5,2);
      vertices[18].Set(-2,2);

      vertices[19].Set(-0.5f,0);
      
      
      
      int count = 20;
      b2PolygonShape polygon;
      polygon.Set(vertices, count);


      b2FixtureDef bmp1;
      bmp1.shape = &polygon;
      bmp1.density = 50.0f;
      bmp1.friction = 0.0f;
      bmp1.restitution = 0.0f;
      b2BodyDef ballbd1;
      ballbd1.type = b2_staticBody;
      ballbd1.position.Set(-30.0f, 30.0f);
      tblp = m_world->CreateBody(&ballbd1);
      tblp->CreateFixture(&bmp1);
      ///////////////////////blimp///////////////////////


      //////////////copters/////////////////////
    
      b2Body* cop1;b2Body* cop2;b2Body* fire1;b2Body* fire2;

      b2Vec2 vertices1[4];
      vertices1[0].Set(0, 0);
      vertices1[1].Set(3, 1);
      vertices1[2].Set(0, 2);
      vertices1[3].Set(-3, 1);

      b2PolygonShape polygon1;
      polygon1.Set(vertices1, 4);  

      b2FixtureDef helis;
      helis.shape = &polygon1;
      helis.density = 50.0f;
      helis.friction = 0.0f;
      helis.restitution = 0.0f;
      b2BodyDef ballbd2;
      ballbd2.type = b2_dynamicBody;
      ballbd2.position.Set(-30.0f, 23.0f);


      cop1 = m_world->CreateBody(&ballbd2);
      cop1->CreateFixture(&helis);    

      ballbd2.position.Set(-30.0f, 20.0f);
      cop2 = m_world->CreateBody(&ballbd2);
      cop2->CreateFixture(&helis);

      ////FIRE///////////////////////////////////////////////
      b2Vec2 verticesfire[4];
      verticesfire[0].Set(0, 0);
      verticesfire[1].Set(2, 0);
      verticesfire[2].Set(2, 2);
      verticesfire[3].Set(0, 2);

      b2PolygonShape polygonfire;
      polygonfire.Set(verticesfire, 4);  

      b2FixtureDef fff;
      fff.shape = &polygonfire;
      fff.density = 50.0f;
      fff.friction = 0.0f;
      fff.restitution = 0.0f;
      b2BodyDef firesquare;
      firesquare.type = b2_staticBody;

      firesquare.position.Set(30.0f, 0.0f);
      fire1 = m_world->CreateBody(&firesquare);
      fire1->CreateFixture(&fff);

      firesquare.position.Set(8.0f, 0.0f);
      fire2 = m_world->CreateBody(&firesquare);
      fire2->CreateFixture(&fff);
    }
    

       
    
  }

  sim_t *sim = new sim_t("Dominos", dominos_t::create);
}
