#include "collision_skeleton.h"
#include "collision_shapes.h"
#include "kinematics/Shape.h"
#include <cmath>




namespace collision_checking{

      

    
CollisionSkeletonNode::CollisionSkeletonNode(kinematics::BodyNode* _bodyNode)
{
    bodyNode = _bodyNode;
    cdmesh = createCube<RSS>(bodyNode->getShape()->getDim()[0], bodyNode->getShape()->getDim()[1], bodyNode->getShape()->getDim()[2]);
    
}
CollisionSkeletonNode::~CollisionSkeletonNode()
{
    delete cdmesh;
}

int CollisionSkeletonNode::checkCollision(CollisionSkeletonNode* otherNode, std::vector<ContactPoint>& result, int num_max_contact)
{
    Eigen::MatrixXd worldTrans(4, 4);
    Eigen::MatrixXd matCOM;
    

    BVH_CollideResult res;
    Vec3f R1[3], T1, R2[3], T2;

//     matCOM.setIdentity(4, 4);
// 
//     for(int k=0;k<3;k++)
//     {
//         matCOM(k, 3) = bodyNode->getLocalCOM()[k];
//     }
//    worldTrans = bodyNode->getWorldTransform()*matCOM;
    evalRT();


//     matCOM.setIdentity(4, 4);
// 
//     for(int k=0;k<3;k++)
//     {
//         matCOM(k, 3) = otherNode->bodyNode->getLocalCOM()[k];
//     }
//     worldTrans = otherNode->bodyNode->getWorldTransform()*matCOM;
    otherNode->evalRT();

    res.num_max_contacts = num_max_contact;
    collide(*cdmesh, mR, mT, *otherNode->cdmesh, otherNode->mR, otherNode->mT, &res);

    for(int i=0;i<res.numPairs();i++)
    {
        ContactPoint pair;
        pair.bd1 = bodyNode;
        pair.bd2 = otherNode->bodyNode;

        pair.bdID1 = this->bodynodeID;
        pair.bdID2 = otherNode->bodynodeID;
        Vec3f v;
        
//         v = res.collidePairs()[i].contact_point;
//         pair.point = Eigen::Vector3d(v[0], v[1], v[2]);
        if(evalContactPosition(res, otherNode, i, pair.point)==false)continue;
        v = res.collidePairs()[i].normal;
        pair.normal = Eigen::Vector3d(v[0], v[1], v[2]);
        result.push_back(pair);

    }

    int collisionNum = result.size();
    return collisionNum;

}

void CollisionSkeletonNode::evalRT()
{
    Eigen::MatrixXd worldTrans(4, 4);
    Eigen::MatrixXd matCOM;
    matCOM.setIdentity(4, 4);
     
    for(int k=0;k<3;k++)
    {
        matCOM(k, 3) = bodyNode->getLocalCOM()[k];
    }
    worldTrans = bodyNode->getWorldTransform()*matCOM;
    //mat.transposeInPlace();
    for(int i=0;i<3;i++)
        for(int j=0;j<3;j++)
            mR[i][j] = worldTrans(j, i);
    for(int i=0;i<3;i++)
        mT[i] = worldTrans(i, 3);
}

bool CollisionSkeletonNode::evalContactPosition( BVH_CollideResult& result,  CollisionSkeletonNode* other, int idx, Eigen::Vector3d& contactPosition )
{
    int id1, id2;
    Triangle tri1, tri2;
    CollisionSkeletonNode* node1 = this;
    CollisionSkeletonNode* node2 = other;
    id1 = result.id1(idx);
    id2 = result.id2(idx);
    tri1 = node1->cdmesh->tri_indices[id1];
    tri2 = node2->cdmesh->tri_indices[id2];

    Vec3f v1, v2, v3, p1, p2, p3;
    v1 = node1->cdmesh->vertices[tri1[0]];
    v2 = node1->cdmesh->vertices[tri1[1]];
    v3 = node1->cdmesh->vertices[tri1[2]];

    p1 = node2->cdmesh->vertices[tri2[0]];
    p2 = node2->cdmesh->vertices[tri2[1]];
    p3 = node2->cdmesh->vertices[tri2[2]];

    Vec3f contact;

    v1 = TransformVertex(v1);
    v2 = TransformVertex(v2);
    v3 = TransformVertex(v3);
    p1 = TransformVertex(p1);
    p2 = TransformVertex(p2);
    p3 = TransformVertex(p3);

    bool testRes;
    testRes = FFtest(v1, v2, v3, p1, p2, p3, contact);
    contactPosition = Eigen::Vector3d(contact[0], contact[1], contact[2]);
    return testRes;
}




SkeletonCollision::~SkeletonCollision()
{
    for(int i=0;i<mCollisionSkeletonNodeList.size();i++)
        delete mCollisionSkeletonNodeList[i];
}

void SkeletonCollision::addCollisionSkeletonNode(kinematics::BodyNode *_bd, bool _bRecursive)
{
    if(_bRecursive == false || _bd->getNumChildJoints() ==0)
    {
        CollisionSkeletonNode* csnode = new CollisionSkeletonNode(_bd);
        csnode->bodynodeID = mCollisionSkeletonNodeList.size();
        mCollisionSkeletonNodeList.push_back(csnode);
    }
    else
    {
        addCollisionSkeletonNode(_bd, false);
        for(int i=0; i<_bd->getNumChildJoints();i++)
            addCollisionSkeletonNode(_bd->getChildNode(i), true);
    }

}

void SkeletonCollision::checkCollision(bool bConsiderGround)
{

    int num_max_contact = 100;
    clearAllContacts();
    
    for(int i=0; i<mCollisionSkeletonNodeList.size();i++)
        for(int j=i+1;j<mCollisionSkeletonNodeList.size();j++)
        {
             if(mCollisionSkeletonNodeList[i]->bodyNode->getParentNode()==mCollisionSkeletonNodeList[j]->bodyNode||
                 mCollisionSkeletonNodeList[j]->bodyNode->getParentNode()==mCollisionSkeletonNodeList[i]->bodyNode)
                  continue;
           
            
            
            mCollisionSkeletonNodeList[i]->checkCollision(mCollisionSkeletonNodeList[j], mContactPointList, num_max_contact);
            
    
        }
}




}
