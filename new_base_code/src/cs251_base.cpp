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

#include "cs251_base.hpp"
#include <cstdio>
#include <vector>
#include <iostream>
#include <unistd.h>
#include <time.h>
#include <stdlib.h>
#include <cmath>
using namespace std;
using namespace cs251;

int fires_created=0; int no_of_fires=6;int cop_created=0;const int no_of_cops=5;
//double dist[no_of_cops][no_of_fires];double assgns[no_of_cops][no_of_fires];// old algo
//double ages[no_of_fires]; // old algo
double speed = 10;
vector<b2Body*> copter_list;vector<b2Body*> fire_list;vector<b2Body*> stations_list;
int initialised=0;

int newok=0,second_cond=0,kk=no_of_cops;
float closeness_limit = 0.5f;
double finish = 0,falsefire=0,ffed=0;

base_sim_t::base_sim_t()
{
	b2Vec2 gravity;
	gravity.Set(0.0f, 0.0f);
	m_world = new b2World(gravity);

	m_text_line = 30;  

	m_point_count = 0;

	m_world->SetDebugDraw(&m_debug_draw);
	
	m_step_count = 0;

	b2BodyDef body_def;
	m_ground_body = m_world->CreateBody(&body_def);

	memset(&m_max_profile, 0, sizeof(b2Profile));
	memset(&m_total_profile, 0, sizeof(b2Profile));
}

base_sim_t::~base_sim_t()
{
	// By deleting the world, we delete the bomb, mouse joint, etc.
	delete m_world;
	m_world = NULL;
}

void base_sim_t::pre_solve(b2Contact* contact, const b2Manifold* oldManifold)
{
  const b2Manifold* manifold = contact->GetManifold();
  
  if (manifold->pointCount == 0)
    {
      return;
    }
  
  b2Fixture* fixtureA = contact->GetFixtureA();
  b2Fixture* fixtureB = contact->GetFixtureB();
  
  b2PointState state1[b2_maxManifoldPoints], state2[b2_maxManifoldPoints];
  b2GetPointStates(state1, state2, oldManifold, manifold);
  
  b2WorldManifold world_manifold;
  contact->GetWorldManifold(&world_manifold);
  
  for (int32 i = 0; i < manifold->pointCount && m_point_count < k_max_contact_points; ++i)
    {
      contact_point_t* cp = m_points + m_point_count;
      cp->fixtureA = fixtureA;
      cp->fixtureB = fixtureB;
      cp->position = world_manifold.points[i];
      cp->normal = world_manifold.normal;
      cp->state = state2[i];
      ++m_point_count;
    }
}

void base_sim_t::draw_title(int x, int y, const char *string)
{
    m_debug_draw.DrawString(x, y, string);
}

  

void base_sim_t::step(settings_t* settings)
{
  float32 time_step = settings->hz > 0.0f ? 1.0f / settings->hz : float32(0.0f);

  if (settings->pause)
    {
      if (settings->single_step)
	{
	  settings->single_step = 0;
	}
      else
	{
	  time_step = 0.0f;
	}
      
      m_debug_draw.DrawString(5, m_text_line, "****PAUSED****");
      m_text_line += 15;
    }
  

  uint32 flags = 0;
  flags += settings->draw_shapes			* b2Draw::e_shapeBit;
  flags += settings->draw_joints			* b2Draw::e_jointBit;
  flags += settings->draw_AABBs			* b2Draw::e_aabbBit;
  flags += settings->draw_pairs			* b2Draw::e_pairBit;
  flags += settings->draw_COMs				* b2Draw::e_centerOfMassBit;
  m_debug_draw.SetFlags(flags);
  
  m_world->SetWarmStarting(settings->enable_warm_starting > 0);
  m_world->SetContinuousPhysics(settings->enable_continuous > 0);
  m_world->SetSubStepping(settings->enable_sub_stepping > 0);
  
  m_point_count = 0;
  
  m_world->Step(time_step, settings->velocity_iterations, settings->position_iterations);

  /***********************CODE HERE*****************************/
  
  b2Body* bdptr = m_world->GetBodyList();
  vector<b2Body*> bodylist;
  while(bdptr->GetNext()){
    bodylist.push_back(bdptr);
    bdptr=bdptr->GetNext();
  }
  //int n=bodylist.size();
//cout<<n<<endl;
//usleep(3000000);
  //////////////    Fires   created    ///////////////    
      if(fires_created==0){

      b2CircleShape circle;
      circle.m_radius = 2.5f;
  
      b2FixtureDef bmp;
      bmp.shape = &circle;
      bmp.density = 50.0f;
      bmp.friction = 0.0f;
      bmp.restitution = 0.0f;
      b2BodyDef ballbd;
      ballbd.type = b2_dynamicBody;
      bmp.filter.categoryBits = 0x0001;
      bmp.filter.maskBits = 0x0000;
      for(int i=0;i<no_of_fires;i++)
      {
        // ages[i]=0; // old algo
        ballbd.position.Set(-35+rand()%70, rand()%40+3);
        b2Body* fire = m_world->CreateBody(&ballbd);
        fire->CreateFixture(&bmp);
      }
    }
    fires_created=1;
////////////////////////////////////////////////////////////
    b2Vec2 zero_vel;
    zero_vel.x=0;zero_vel.y=0;

 /////////////////    Copters    and   Stations/////////////////////////

    if(cop_created==0){

      b2Vec2 cop_poly[4];
      cop_poly[0].Set(0, 0);
      cop_poly[1].Set(1, 1);
      cop_poly[2].Set(0, 2);
      cop_poly[3].Set(-1, 1);

      b2PolygonShape cop_sss;
      cop_sss.Set(cop_poly, 4);  
  
      b2FixtureDef bmp;
      bmp.shape = &cop_sss;
      bmp.density = 50.0f;
      bmp.friction = 0.0f;
      bmp.restitution = 0.0f;
      b2BodyDef ballbd;
      ballbd.type = b2_dynamicBody;
      bmp.filter.categoryBits = 0x0010;
      bmp.filter.maskBits = 0x0000;// change mask to avoid self collision 

      b2CircleShape circle;
      circle.m_radius = 0.5f;//station size

      b2FixtureDef tmp;
      tmp.shape = &circle;
      tmp.density = 50.0f;
      tmp.friction = 0.0f;
      tmp.restitution = 0.0f;
      tmp.filter.categoryBits = 0x0010;
      tmp.filter.maskBits = 0x0000;// change mask to avoid self collision 


      for(int i=0;i<no_of_cops;i++)
      {
        double x=-35+rand()%70;
        ballbd.position.Set(x,0);
        b2Body* fire = m_world->CreateBody(&ballbd);
        fire->CreateFixture(&bmp);
      }
      for(int i=0;i<no_of_cops;i++)//stations
      {
        double x=-35+rand()%70;
        ballbd.position.Set(x,0);
        b2Body* fre = m_world->CreateBody(&ballbd);
        fre->CreateFixture(&tmp);
      }
    }
    cop_created=1;

//////////////////////////////////////////////

//fires->cops->stations
  
    

////////////   initialisation   ///////////////
if(initialised == 1){

  for(int i=0;i<no_of_cops;i++)
    {
      b2Body* g=bodylist[i];
      //cout<<g->GetWorldCenter().x<<endl;
      stations_list.push_back(g);

    }

  for(int i=0;i<no_of_cops;i++)
    {
      b2Body* g=bodylist[i+no_of_cops];
      //cout<<g->GetWorldCenter().x<<endl;
      copter_list.push_back(g);

    }

  for(int i=0;i<no_of_fires;i++)
    {
      b2Body* g=bodylist[2*no_of_cops+ i];
      //cout<<g->GetWorldCenter().x<<endl;
      fire_list.push_back(g);

    }

  // for(int i=0;i<no_of_cops;i++)  // old algo
  //   for(int j=0;j<no_of_fires;j++)
  //       dist[i][j]=b2Distance(copter_list[i]->GetWorldCenter(),fire_list[j]->GetWorldCenter());
}
    initialised++;
///////////////////  initialised = 1 ending ..../////////////////////////////

// for(int i=0;i<no_of_fires;i++)
//   ages[i]++;

// for (int i = 0; i < no_of_cops; ++i)
// {
//   for (int j= 0; j < no_of_fires; ++j)
//   {
//     assgns[i][j]=ages[j]+dist[i][j]/2;
//   }
// }

///////////////////////////////ASSIGNMENTS ABOVE /////////////////////
if(initialised > 1)
{
  
  /*****  NORMAL NO ALGO MODE ********
  for(int i=0;i<no_of_cops;i++)
  {
    double max_i=-1;int m_index=0;
    for(int j=0;j<no_of_fires;j++)
    {
      if(assgns[i][j]>max_i)
        {
          m_index=j;max_i=assgns[i][j];
        }
    }
    b2Vec2 rvector=fire_list[m_index]->GetWorldCenter()-copter_list[i]->GetWorldCenter();
    double xx=rvector.x;double yy=rvector.y;
    //cout<<xx<<" "<<yy<<endl;
    rvector.x=speed*xx/sqrt(xx*xx+yy*yy);
    rvector.y=speed*yy/sqrt(xx*xx+yy*yy);
    copter_list[i]->SetLinearVelocity(rvector);
  }


  for(int i=0;i<no_of_fires;i++)
  {
    for (int j = 0; j < no_of_cops; ++j)
    {
      if(b2Distance(fire_list[i]->GetWorldCenter(),copter_list[j]->GetWorldCenter())< 2)
        ages[i]=0;
    }
  }
  **********************************************/


  /***************   MY ALGO  *******************/
  if(no_of_cops >= no_of_fires && newok==0)
  {
    for(int i=0;i<no_of_fires;i++)
    {
      b2Vec2 rvector=fire_list[i]->GetWorldCenter()-copter_list[i]->GetWorldCenter();
      double xx=rvector.x;double yy=rvector.y;
      rvector.x=speed*xx/sqrt(xx*xx+yy*yy);
      rvector.y=speed*yy/sqrt(xx*xx+yy*yy);
      copter_list[i]->SetLinearVelocity(rvector);
    }
    newok=0;
  }

  else
  {

    if(second_cond == 0)
    {
      for(int i=0;i<no_of_cops;i++)
      {
        b2Vec2 rvector=fire_list[i]->GetWorldCenter()-copter_list[i]->GetWorldCenter();
        double xx=rvector.x;double yy=rvector.y;
        rvector.x=speed*xx/sqrt(xx*xx+yy*yy);
        rvector.y=speed*yy/sqrt(xx*xx+yy*yy);
        copter_list[i]->SetLinearVelocity(rvector);
      }
    }
    second_cond = 1;
    if(second_cond == 1)
    {
      for(int i=0;i<no_of_cops;i++)
      {
        //cout<<"A"<<endl;
        for(int j=0;j<no_of_fires;j++)
        {
          if(b2Distance(fire_list[j]->GetWorldCenter(),copter_list[i]->GetWorldCenter())< closeness_limit)
          {
            //cout<<"B"<<endl;
            b2Vec2 rvector=fire_list[kk%no_of_fires]->GetWorldCenter()-copter_list[i]->GetWorldCenter();
            double xx=rvector.x;double yy=rvector.y;
            rvector.x=speed*xx/sqrt(xx*xx+yy*yy);
            rvector.y=speed*yy/sqrt(xx*xx+yy*yy);
            copter_list[i]->SetLinearVelocity(rvector);
            kk++;
          }
        }
      }
    }


  }
}


/****** FALSE FIRE  ********/
falsefire++;
//cout<<falsefire <<endl;
if( falsefire > 500 && ffed==0)
{
  while(fire_list.size()>0)
    fire_list.pop_back();


  int k=rand()%no_of_fires;
  cout<<"K "<<k<<endl;
  double pox=fire_list[k]->GetWorldCenter().x;
  double poy=fire_list[k]->GetWorldCenter().y;
  m_world->DestroyBody(fire_list[k]);
  no_of_fires--;

  b2Vec2 cop_poly[4];
  cop_poly[0].Set(0, 0);
  cop_poly[1].Set(4, 0);
  cop_poly[2].Set(4, 4);
  cop_poly[3].Set(0, 4);

  b2PolygonShape cop_sss;
  cop_sss.Set(cop_poly, 4);  
  
  b2FixtureDef bmp;
  bmp.shape = &cop_sss;
  bmp.density = 50.0f;
  bmp.friction = 0.0f;
  bmp.restitution = 0.0f;
  b2BodyDef ballbd;
  ballbd.type = b2_dynamicBody;
  bmp.filter.categoryBits = 0x0010;
  bmp.filter.maskBits = 0x0000;// change mask to avoid self collision 

  ballbd.position.Set(pox,poy);
  b2Body* fire = m_world->CreateBody(&ballbd);
  fire->CreateFixture(&bmp);
  ffed=1;
  second_cond=0;

  for(int i=0;i<no_of_cops;i++)
    copter_list[i]->SetLinearVelocity(zero_vel);

  // for(int i=0;i<bodylist.size();i++)
  // {
  //   cout<<bodylist[i]->GetWorldCenter().x<< " "<<bodylist[i]->GetWorldCenter().y<<endl;
  // }
  // cout<<"DSF"<<endl;
  
  for(int i=0;i<=no_of_fires;i++)
  {
    b2Body* g=bodylist[2*no_of_cops+i];
    if(i!=k)
      fire_list.push_back(g);
  }
  // cout<<"----------------------"<<endl;
  // for(int i=0;i<no_of_fires;i++)
  //   cout<<fire_list[i]->GetWorldCenter().x<<" "<<fire_list[i]->GetWorldCenter().y<<endl;
}

/***************************/






/****  Comment below part to continue simulation forever   ****/
finish++;
//cout<<finish<<endl;
if(finish > 2000) //~ 30 secs -- 4000
{
  for(int i=0;i<no_of_cops;i++)
  {
    b2Vec2 rvector=stations_list[i]->GetWorldCenter()-copter_list[i]->GetWorldCenter();
    double xx=rvector.x;double yy=rvector.y;
    rvector.x=speed*xx/sqrt(xx*xx+yy*yy);
    rvector.y=speed*yy/sqrt(xx*xx+yy*yy);
    copter_list[i]->SetLinearVelocity(rvector);
  }
  initialised = -1;
}
/**************************************************************/





/************************END MY ALGO**************************************/





  /***********************   END  CODING   *****************************/
  m_world->DrawDebugData();
  
  if (time_step > 0.0f)
    {
      ++m_step_count;
    }
  
  if (settings->draw_stats)
    {
      int32 body_count = m_world->GetBodyCount();
      int32 contact_count = m_world->GetContactCount();
      int32 joint_count = m_world->GetJointCount();
      m_debug_draw.DrawString(5, m_text_line, "bodies/contacts/joints = %d/%d/%d", body_count, contact_count, joint_count);
      m_text_line += 15;
      
      int32 proxy_count = m_world->GetProxyCount();
      int32 height = m_world->GetTreeHeight();
      int32 balance = m_world->GetTreeBalance();
      float32 quality = m_world->GetTreeQuality();
      m_debug_draw.DrawString(5, m_text_line, "proxies/height/balance/quality = %d/%d/%d/%g", proxy_count, height, balance, quality);
      m_text_line += 15;
    }
  
  // Track maximum profile times
  {
    const b2Profile& p = m_world->GetProfile();
    m_max_profile.step = b2Max(m_max_profile.step, p.step);
    m_max_profile.collide = b2Max(m_max_profile.collide, p.collide);
    m_max_profile.solve = b2Max(m_max_profile.solve, p.solve);
    m_max_profile.solveInit = b2Max(m_max_profile.solveInit, p.solveInit);
    m_max_profile.solveVelocity = b2Max(m_max_profile.solveVelocity, p.solveVelocity);
    m_max_profile.solvePosition = b2Max(m_max_profile.solvePosition, p.solvePosition);
    m_max_profile.solveTOI = b2Max(m_max_profile.solveTOI, p.solveTOI);
    m_max_profile.broadphase = b2Max(m_max_profile.broadphase, p.broadphase);
    
    m_total_profile.step += p.step;
    m_total_profile.collide += p.collide;
    m_total_profile.solve += p.solve;
    m_total_profile.solveInit += p.solveInit;
    m_total_profile.solveVelocity += p.solveVelocity;
    m_total_profile.solvePosition += p.solvePosition;
    m_total_profile.solveTOI += p.solveTOI;
    m_total_profile.broadphase += p.broadphase;
  }
  
  if (settings->draw_profile)
    {
      const b2Profile& p = m_world->GetProfile();
      
      b2Profile ave_profile;
      memset(&ave_profile, 0, sizeof(b2Profile));
      if (m_step_count > 0)
	{
	  float32 scale = 1.0f / m_step_count;
	  ave_profile.step = scale * m_total_profile.step;
	  ave_profile.collide = scale * m_total_profile.collide;
	  ave_profile.solve = scale * m_total_profile.solve;
	  ave_profile.solveInit = scale * m_total_profile.solveInit;
	  ave_profile.solveVelocity = scale * m_total_profile.solveVelocity;
	  ave_profile.solvePosition = scale * m_total_profile.solvePosition;
	  ave_profile.solveTOI = scale * m_total_profile.solveTOI;
	  ave_profile.broadphase = scale * m_total_profile.broadphase;
	}
      
      m_debug_draw.DrawString(5, m_text_line, "step [ave] (max) = %5.2f [%6.2f] (%6.2f)", p.step, ave_profile.step, m_max_profile.step);
      m_text_line += 15;
      m_debug_draw.DrawString(5, m_text_line, "collide [ave] (max) = %5.2f [%6.2f] (%6.2f)", p.collide, ave_profile.collide, m_max_profile.collide);
      m_text_line += 15;
      m_debug_draw.DrawString(5, m_text_line, "solve [ave] (max) = %5.2f [%6.2f] (%6.2f)", p.solve, ave_profile.solve, m_max_profile.solve);
      m_text_line += 15;
      m_debug_draw.DrawString(5, m_text_line, "solve init [ave] (max) = %5.2f [%6.2f] (%6.2f)", p.solveInit, ave_profile.solveInit, m_max_profile.solveInit);
      m_text_line += 15;
      m_debug_draw.DrawString(5, m_text_line, "solve velocity [ave] (max) = %5.2f [%6.2f] (%6.2f)", p.solveVelocity, ave_profile.solveVelocity, m_max_profile.solveVelocity);
      m_text_line += 15;
      m_debug_draw.DrawString(5, m_text_line, "solve position [ave] (max) = %5.2f [%6.2f] (%6.2f)", p.solvePosition, ave_profile.solvePosition, m_max_profile.solvePosition);
      m_text_line += 15;
      m_debug_draw.DrawString(5, m_text_line, "solveTOI [ave] (max) = %5.2f [%6.2f] (%6.2f)", p.solveTOI, ave_profile.solveTOI, m_max_profile.solveTOI);
      m_text_line += 15;
      m_debug_draw.DrawString(5, m_text_line, "broad-phase [ave] (max) = %5.2f [%6.2f] (%6.2f)", p.broadphase, ave_profile.broadphase, m_max_profile.broadphase);
      m_text_line += 15;
    }
    
  if (settings->draw_contact_points)
    {
      //const float32 k_impulseScale = 0.1f;
      const float32 k_axis_scale = 0.3f;
      
      for (int32 i = 0; i < m_point_count; ++i)
	{
	  contact_point_t* point = m_points + i;
	  
	  if (point->state == b2_addState)
	    {
	      // Add
	      m_debug_draw.DrawPoint(point->position, 10.0f, b2Color(0.3f, 0.95f, 0.3f));
	    }
	  else if (point->state == b2_persistState)
	    {
	      // Persist
	      m_debug_draw.DrawPoint(point->position, 5.0f, b2Color(0.3f, 0.3f, 0.95f));
	    }
	  
	  if (settings->draw_contact_normals == 1)
	    {
	      b2Vec2 p1 = point->position;
	      b2Vec2 p2 = p1 + k_axis_scale * point->normal;
	      m_debug_draw.DrawSegment(p1, p2, b2Color(0.9f, 0.9f, 0.9f));
	    }
	  else if (settings->draw_contact_forces == 1)
	    {
	      //b2Vec2 p1 = point->position;
	      //b2Vec2 p2 = p1 + k_forceScale * point->normalForce * point->normal;
	      //DrawSegment(p1, p2, b2Color(0.9f, 0.9f, 0.3f));
	    }
	  
	  if (settings->draw_friction_forces == 1)
	    {
	      //b2Vec2 tangent = b2Cross(point->normal, 1.0f);
	      //b2Vec2 p1 = point->position;
	      //b2Vec2 p2 = p1 + k_forceScale * point->tangentForce * tangent;
	      //DrawSegment(p1, p2, b2Color(0.9f, 0.9f, 0.3f));
	    }
	}
    }
}


  // PATH CROSS ALGO
  // for(int t=0;t<no_of_cops;t++)
  // {
  //   for(int tt=0;tt<no_of_cops;tt++)
  //   {
  //     if(t!=tt && b2Distance(copter_list[t]->GetWorldCenter(),copter_list[tt]->GetWorldCenter()) < 2.f)
  //     {
  //       int mm=0,ind=0;
  //       for(int h=0;h<no_of_fires;h++)
  //       {
  //         if(b2Distance(copter_list[tt]->GetWorldCenter(),fire_list[h]->GetWorldCenter())>mm)
  //         {
  //           ind=h;mm=b2Distance(copter_list[tt]->GetWorldCenter(),fire_list[h]->GetWorldCenter());
  //         }
  //       }


  //       b2Vec2 rvector=fire_list[ind]->GetWorldCenter()-copter_list[tt]->GetWorldCenter();
  //       double xx=rvector.x;double yy=rvector.y;
  //       rvector.x=speed*xx/sqrt(xx*xx+yy*yy);
  //       rvector.y=speed*yy/sqrt(xx*xx+yy*yy);
  //       copter_list[tt]->SetLinearVelocity(rvector);

  //       int y=0;
  //       while(y<700)
  //         y++;


  //       b2Vec2 rvector1=fire_list[rand()%no_of_fires]->GetWorldCenter()-copter_list[t]->GetWorldCenter();
  //       xx=rvector1.x;yy=rvector1.y;
  //       rvector1.x=speed*xx/sqrt(xx*xx+yy*yy);
  //       rvector1.y=speed*yy/sqrt(xx*xx+yy*yy);
  //       copter_list[t]->SetLinearVelocity(rvector1);
  //     }
  //   }
  // }
