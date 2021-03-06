#include "RigidMeshDeformer2D.h"

#include <WmlLinearSystem.h>
#include <WmlMatrix4.h>
#include "WmlExtTriangleUtils.h"
#include "rmsdebug.h"
#include "cula.h"

using namespace rmsmesh;

#define imin(X, Y)  ((X) < (Y) ? (X) : (Y))

cv::Mat toCv(Wml::GVectord& gvec) {
	return cv::Mat(gvec.GetSize(), 1, CV_64FC1, (double*) gvec);
}

cv::Mat toCv(Wml::GMatrixd& gmat)
{
	return cv::Mat(gmat.GetRows(), gmat.GetColumns(), CV_64FC1, (double*) gmat);
}

// Rigid Mesh Deformation Local memory buffers.
float RigidMeshDeformer2D::A[800*800];
float RigidMeshDeformer2D::U[800*800];
float RigidMeshDeformer2D::S[800];
float RigidMeshDeformer2D::VT[800*800];

void invert(Wml::GMatrixd& from, Wml::GMatrixd& to)
{
	cout << "inverting " << from.GetRows() << " x " << from.GetColumns() << endl;

	size_t time_start, time_end;
	time_start = ofGetElapsedTimeMicros();

	//*
	int rows = from.GetRows();
	int cols = from.GetColumns();

	int len = rows*cols;

	// Note that although we are converting from column major to row major ordering and back, it should not matter, because the A^{-1} = ((A^{T})^{-1})

	RigidMeshDeformer2D::copyDoubleToFloat((double*) from, RigidMeshDeformer2D::A, len);
	
	RigidMeshDeformer2D::setToIdentity(RigidMeshDeformer2D::U, rows);

	int i[800];
	culaSgesv(rows, rows, RigidMeshDeformer2D::A, rows, i, RigidMeshDeformer2D::U, rows);
	
	RigidMeshDeformer2D::copyFloatToDouble(RigidMeshDeformer2D::U, (double *) to, len);

	//*/

	/*
	cv::Mat fromMat = toCv(from), toMat = toCv(to);
	cv::invert(fromMat, toMat, cv::DECOMP_LU); // LU is 3x as fast as CHOLESKY
	//*/	

	time_end = ofGetElapsedTimeMicros();

	cout << "inverted. elapsed time = " << time_end - time_start  << endl;
}

inline void RigidMeshDeformer2D::copyDoubleToFloat(double * in, float * out, int len)
{
	for(int i = 0; i < len; i++)
	{
		out[i] = (float)in[i];
	}
}

inline void RigidMeshDeformer2D::copyFloatToDouble(float * in, double * out, int len)
{
	for(int i = 0; i < len; i++)
	{
		out[i] = (double)in[i];
	}
}

inline void RigidMeshDeformer2D::setToIdentity(float * mat, int size)
{
	int index = 0;

	for(int r = 0; r < size; r++)
	for(int c = 0; c < size; c++)
	{
		if(r == c)
		{
			mat[index] = 1;
		}
		else
		{
			mat[index] = 0;
		}

		index++;
	}
}


	
RigidMeshDeformer2D::RigidMeshDeformer2D()
{
	InvalidateSetup();

	/*
	int M = 400;
	int N = 400;

	int LDU = 400;
	int LDVT = 400; 

	A = (float*)malloc(M*N*sizeof(float));
    S = (float*)malloc(imin(M,N)*sizeof(float));
    U = (float*)malloc(LDU*M*sizeof(float));
    VT = (float*)malloc(LDVT*N*sizeof(float));
	*/

}

void RigidMeshDeformer2D::SetDeformedHandle( unsigned int nHandle, const ofVec2f & vHandle )
{
	Constraint c(nHandle, vHandle);
	UpdateConstraint(c);
}

void RigidMeshDeformer2D::RemoveHandle( unsigned int nHandle )
{	
	Constraint c(nHandle, ofVec2f(0,0));
	m_vConstraints.erase(c);
	m_vDeformedVerts[nHandle].vPosition = m_vInitialVerts[nHandle].vPosition;
	InvalidateSetup();
}



 void BarycentricCoords( const ofVec2f & vTriVtx1, 
 const ofVec2f & vTriVtx2,
 const ofVec2f & vTriVtx3,
 const ofVec2f & vVertex,
 float & fBary1, float & fBary2, float & fBary3 )
 {
 
     ofVec2f kV02 = vTriVtx1 - vTriVtx3;
     ofVec2f kV12 = vTriVtx2 - vTriVtx3;
     ofVec2f kPV2 = vVertex - vTriVtx3;
     
     float fM00 = kV02.dot(kV02);
     float fM01 = kV02.dot(kV12);
     float fM11 = kV12.dot(kV12);
     float fR0 = kV02.dot(kPV2);
     float fR1 = kV12.dot(kPV2);
     float fDet = fM00*fM11 - fM01*fM01;
     //    ASSERT( Wml::Math<Real>::FAbs(fDet) > (Real)0.0 );
     float fInvDet = ((float)1.0)/fDet;
     
     fBary1 = (fM11*fR0 - fM01*fR1)*fInvDet;
     fBary2 = (fM00*fR1 - fM01*fR0)*fInvDet;
     fBary3 = (float)1.0 - fBary1 - fBary2;
 }



void RigidMeshDeformer2D::UnTransformPoint( ofVec2f & vTransform )
{
	// find triangle
	size_t nTris = m_vTriangles.size();
	for ( unsigned int i = 0; i < nTris; ++i ) {
		ofVec2f v1( m_vDeformedVerts[m_vTriangles[i].nVerts[0]].vPosition );
		ofVec2f v2( m_vDeformedVerts[m_vTriangles[i].nVerts[1]].vPosition );
		ofVec2f v3( m_vDeformedVerts[m_vTriangles[i].nVerts[2]].vPosition );
		
		float fBary1, fBary2, fBary3;
		BarycentricCoords(v1,v2,v3,vTransform, fBary1, fBary2, fBary3 );
        
		if ( fBary1 < 0 || fBary1 > 1 || fBary2 < 0 || fBary2 > 1 || fBary3 < 0 || fBary3 > 1 )
			continue;
		
		ofVec2f v1Init( m_vInitialVerts[m_vTriangles[i].nVerts[0]].vPosition );
		ofVec2f v2Init( m_vInitialVerts[m_vTriangles[i].nVerts[1]].vPosition );
		ofVec2f v3Init( m_vInitialVerts[m_vTriangles[i].nVerts[2]].vPosition );
		vTransform = fBary1 * v1Init + fBary2 * v2Init + fBary3 * v3Init;
		return;
	}
	
}


void RigidMeshDeformer2D::InitializeFromMesh( ofMesh * pMesh )
{
	m_vConstraints.clear();
	m_vInitialVerts.resize(0);
	m_vDeformedVerts.resize(0);
	m_vTriangles.resize(0);
	
	// copy vertices
	unsigned int nVerts = pMesh->getVertices().size();
	for ( unsigned int i = 0; i < nVerts; ++i ) {
		//Wml::Vector3f vVertex;
		

        // TODO THIS IS OK?
        ofVec3f vVertex = pMesh->getVertices()[i];
        
		Vertex v;
		v.vPosition = ofVec2f( vVertex.x, vVertex.y);
		m_vInitialVerts.push_back( v );
		m_vDeformedVerts.push_back( v );
	}
	
    
    // TODO here?

	// copy triangles
	unsigned int nTris = pMesh->getIndices().size() / 3;
	for ( unsigned int i = 0; i < nTris; ++i ) {
		Triangle t;
        
        
        t.vTriCoords[0].set( pMesh->getVertices()[ pMesh->getIndices()[i*3]]);
        t.vTriCoords[1].set( pMesh->getVertices()[ pMesh->getIndices()[i*3+1]]);
        t.vTriCoords[2].set( pMesh->getVertices()[ pMesh->getIndices()[i*3+2]]);
        
        t.nVerts[0] = pMesh->getIndices()[i*3];
        t.nVerts[1] = pMesh->getIndices()[i*3+1];
        t.nVerts[2] = pMesh->getIndices()[i*3+2];
        
        
		//pMesh->GetTriangle(i, t.nVerts );
		m_vTriangles.push_back(t);
	}
	
	
	// set up triangle-local coordinate systems
	for ( unsigned int i = 0; i < nTris; ++i ) {
		Triangle & t = m_vTriangles[i];
		
		for ( int j = 0; j < 3; ++j ) {
			unsigned int n0 = j;
			unsigned int n1 = (j+1)%3;
			unsigned int n2 = (j+2)%3;
			
			
			
			
			ofPoint v0 = (GetInitialVert( t.nVerts[n0] ));
			ofPoint v1 = (GetInitialVert( t.nVerts[n1] ));
			ofPoint v2 = (GetInitialVert( t.nVerts[n2] ));
			
			
			// find coordinate system
			ofPoint v01( v1 - v0 );
			ofPoint v01N( v01 );  v01N.normalize();
			ofPoint v01Rot90( v01.y, -v01.x );
			ofPoint v01Rot90N( v01Rot90 );  v01Rot90N.normalize();
			
			// express v2 in coordinate system
			ofPoint vLocal(v2 - v0);
			float fX = vLocal.dot(v01) /  v01.lengthSquared();
			float fY = vLocal.dot(v01Rot90) / v01Rot90.lengthSquared();
			
			
			
			// sanity check
			ofPoint v2test(v0 + fX * v01 + fY * v01Rot90);
			float fLength = (v2test - v2).length();
			if ( fLength > 0.001f )
				Debugbreak();
			
			t.vTriCoords[j].x= fX;
			t.vTriCoords[j].y= fY;
			
			//= fromOF(ofPoint(fX,fY));
			
			
			// find coordinate system
//			ofVec2f v01;
//			v01.x= v1.x- v0.x;
//			v01.y= v1.y- v0.y;
//			
//			cout << n0 << " n0 " << t.nVerts[n0] << " " << m_vInitialVerts[ t.nVerts[n0]  ].vPosition.x<< " " << m_vInitialVerts[ t.nVerts[n0] ].vPosition.y<< endl;
//			cout << n1 << " n1 " << t.nVerts[n1] << " " << m_vInitialVerts[ t.nVerts[n1]  ].vPosition.x<< " " << m_vInitialVerts[ t.nVerts[n1] ].vPosition.y<< endl;
//			
//			cout << v0.x<< " " <<  v1.x<< endl;
//			cout << v01.X ()  << endl;
//			
//			ofVec2f v01N = v01;  v01N.Normalize();
//			
//			ofVec2f v01Rot90( v01.y, -v01.x);
//			ofVec2f v01Rot90N = v01Rot90;  v01Rot90N.Normalize();
//			
//			// express v2 in coordinate system
//			ofVec2f vLocal(v2 - v0);
//			
//			vLocal.x= v2.x- v0.x;
//			vLocal.y= v2.y- v0.y;
//			
//			cout << v01Rot90.x<< " v01Rot90  " << v01Rot90.y<< " " << v01Rot90.lengthsq()  << endl;
//			
//			float fX = vLocal.Dot(v01) /  v01.lengthsq();
//			float fY = vLocal.Dot(v01Rot90) / v01Rot90.lengthsq();
//			
//			cout << vLocal.Dot(v01) << endl;
//			cout << fX << "  fx  " << fY << endl;
//			
//			// sanity check
//			ofVec2f v2test(v0 + fX * v01 + fY * v01Rot90);
//			float fLength = (v2test - v2).Length();
//			if ( fLength > 0.001f )
//				DebugBreak();
//			
			//t.vTriCoords[j] = ofVec2f(fX,fY);
			
			//
			
		}
	}
	
	//std::exit(0);
}


void RigidMeshDeformer2D::UpdateDeformedMesh( ofMesh * pMesh, bool bRigid )
{
	ValidateDeformedMesh(bRigid);
	

    
	std::vector<Vertex> & vVerts = (m_vConstraints.size() > 1) ? m_vDeformedVerts : m_vInitialVerts;
	
	unsigned int nVerts = pMesh->getVertices().size();
	for ( unsigned int i = 0; i < nVerts; ++i ) {
        
		ofVec2f vNewPos; 
		vNewPos.x=  pMesh->getVertices()[i].x;
		vNewPos.y=  pMesh->getVertices()[i].y;
		//cout << "setting update " << vNewPos.x<< " " << vNewPos.y<< endl;
		
        pMesh->getVertices()[i].set( ofPoint(m_vDeformedVerts[i].vPosition.x, m_vDeformedVerts[i].vPosition.y, 0.0f ));
        
        //Wml::Vector3f( vNewPos.x, vNewPos.y, 0.0f ) );
	}
}



void RigidMeshDeformer2D::UpdateConstraint( Constraint & cons )
{
	std::set<Constraint>::iterator found( m_vConstraints.find(cons) );
	if ( found != m_vConstraints.end() ) {
		
		//printf("updating constraint ! \n");
		Constraint copy = *found;
		//... // update member value on copy, varies
		copy.vConstrainedPos = cons.vConstrainedPos;
		
		m_vConstraints.erase(found);
		m_vConstraints.insert(copy);
		
		
		m_vDeformedVerts[cons.nVertex].vPosition = cons.vConstrainedPos;
		
	} else {
        
        //printf("adding constraint ! \n");
		
        
		m_vConstraints.insert( cons );
		m_vDeformedVerts[cons.nVertex].vPosition = cons.vConstrainedPos;
		InvalidateSetup();
	} 
	
}




void ExtractSubMatrix( Wml::GMatrixd & mFrom, int nRowOffset, int nColOffset, Wml::GMatrixd & mTo )
{
	int nRows = mTo.GetRows();
	int nCols = mTo.GetColumns();
	
	for ( int i = 0; i < nRows; ++i ) {
		for ( int j = 0; j < nCols; ++j ) {
			mTo[i][j] = mFrom[ i + nRowOffset][ j + nColOffset ];
		}
	}
}


void RigidMeshDeformer2D::ValidateSetup()
{
	if  ( m_bSetupValid || m_vConstraints.size() < 2)
		return;
	
	
	//printf("Computing matrices for mesh with %i verts....this might take a while...\n", m_vInitialVerts.size() );
	
	PrecomputeOrientationMatrix();
	
	// ok, now scale triangles
	size_t nTris = m_vTriangles.size();
	for ( unsigned int i = 0; i < nTris; ++i )
		PrecomputeScalingMatrices(i);
	
	PrecomputeFittingMatrices();
	
	//printf("Done!\n" );
	
	
	m_bSetupValid = true;
}



// Precomputations enable real time animation.
void RigidMeshDeformer2D::PrecomputeFittingMatrices()
{
	// put constraints into vector (will be useful)
	std::vector<Constraint> vConstraintsVec;
	std::set<Constraint>::iterator cur(m_vConstraints.begin()), end(m_vConstraints.end());
	while ( cur != end )
		vConstraintsVec.push_back( *cur++ );
	
	// resize matrix and clear to zero
	unsigned int nVerts = (unsigned int)m_vDeformedVerts.size();
	size_t nConstraints = vConstraintsVec.size();
	unsigned int nFreeVerts = nVerts - nConstraints;
	
	// figure out vertex ordering. first do free vertices, then constraints
	unsigned int nRow = 0;
	m_vVertexMap.resize(nVerts);
	for ( unsigned int i = 0; i < nVerts; ++i ) {
		Constraint c(i, ofVec2f(0,0));
		if ( m_vConstraints.find(c) != m_vConstraints.end() )
			continue;
		m_vVertexMap[i] = nRow++;
	}
	if ( nRow != nFreeVerts )	Debugbreak();
	for ( unsigned int i = 0 ; i < nConstraints; ++i )
		m_vVertexMap[vConstraintsVec[i].nVertex ] = nRow++;
	if ( nRow != nVerts )	Debugbreak();		// bad!
	
	
	// test vector...
	Wml::GVectord gUTestX( nVerts ), gUTestY(nVerts);
	for ( unsigned int i = 0; i < nVerts; ++i ) {
		Constraint c(i,ofVec2f(0,0));
		if ( m_vConstraints.find(c) != m_vConstraints.end() )
			continue;
		int nRow = m_vVertexMap[i];
		gUTestX[nRow] = m_vInitialVerts[i].vPosition.x;
		gUTestY[nRow] = m_vInitialVerts[i].vPosition.y;
	}
	for ( unsigned int i = 0; i < nConstraints; ++i ) {
		int nRow = m_vVertexMap[ vConstraintsVec[i].nVertex ];
		gUTestX[nRow] = vConstraintsVec[i].vConstrainedPos.x;
		gUTestY[nRow] = vConstraintsVec[i].vConstrainedPos.y;
	}
	
	
	// make Hy and Hx matrices
	Wml::GMatrixd mHX( nVerts, nVerts );
	Wml::GMatrixd mHY( nVerts, nVerts );
	for ( unsigned int i = 0; i < nVerts; ++i )
		for ( unsigned int j = 0; j < nVerts; ++j )
			mHX(i,j) = mHY(i,j) = 0.0;
	
	// ok, now fill matrix
	size_t nTriangles = m_vTriangles.size();
	for ( unsigned int i = 0; i < nTriangles; ++i ) {
		Triangle & t = m_vTriangles[i];
		
		//		_RMSInfo("Triangle %d: \n", i);
		double fTriSumErr = 0;
		for ( int j = 0; j < 3; ++j ) {
			double fTriErr = 0;
			
			int nA = m_vVertexMap[ t.nVerts[j] ];
			int nB = m_vVertexMap[ t.nVerts[(j+1)%3] ];
			
			// X elems
			mHX[nA][nA] += 2;
			mHX[nA][nB] += -2;
			mHX[nB][nA] += -2;
			mHX[nB][nB] += 2;
			
			//  Y elems
			mHY[nA][nA] += 2;
			mHY[nA][nB] += -2;
			mHY[nB][nA] += -2;
			mHY[nB][nB] += 2;			
		}
	}
	
	// extract HX00 and  HY00 matrices
	Wml::GMatrixd mHX00( (int)nFreeVerts, (int)nFreeVerts );
	Wml::GMatrixd mHY00( (int)nFreeVerts, (int)nFreeVerts );
	ExtractSubMatrix( mHX, 0, 0, mHX00 );
	ExtractSubMatrix( mHY, 0, 0, mHY00 );
	
	// Extract HX01 and HX10 matrices
	Wml::GMatrixd mHX01( (int)nFreeVerts, (int)nConstraints );
	Wml::GMatrixd mHX10( (int)nConstraints, (int)nFreeVerts );
	ExtractSubMatrix( mHX, 0, nFreeVerts, mHX01 );
	ExtractSubMatrix( mHX, nFreeVerts, 0, mHX10 );
	
	// Extract HY01 and HY10 matrices
	Wml::GMatrixd mHY01( (int)nFreeVerts, (int)nConstraints );
	Wml::GMatrixd mHY10( (int)nConstraints, (int)nFreeVerts );
	ExtractSubMatrix( mHY, 0, nFreeVerts, mHY01 );
	ExtractSubMatrix( mHY, nFreeVerts, 0, mHY10 );
	
	// now compute HXPrime = HX00 + Transpose(HX00) (and HYPrime)
	//Wml::GMatrixd mHXPrime( mHX00 + mHX00.Transpose() );
	//Wml::GMatrixd mHYPrime( mHY00 + mHY00.Transpose() );
	m_mHXPrime = mHX00;
	m_mHYPrime = mHY00;
	
	// and then D = HX01 + Transpose(HX10)
	//Wml::GMatrixd mDX = mHX01 + mHX10.Transpose();
	//Wml::GMatrixd mDY = mHY01 + mHY10.Transpose();
	m_mDX = mHX01;
	m_mDY = mHY01;
	
	
	// pre-compute LU decompositions
	/*
	bool bResult = Wml::LinearSystemExtd::LUDecompose( m_mHXPrime, m_mLUDecompX );
	if (!bResult)
		DebugBreak();
	bResult = Wml::LinearSystemExtd::LUDecompose( m_mHYPrime, m_mLUDecompY );
	if (!bResult)
		DebugBreak();
	 */
	
	// -- Precompute the Singular value decomposition.

	// CPU.
	cout << "SVD decomposition" << endl;
	size_t time_start = ofGetElapsedTimeMicros();

	
	int rows    = m_mHXPrime.GetRows();
	int columns = m_mHXPrime.GetColumns();

	//cv::Mat m1 = cv::Mat(rows, columns, CV_64F, double(0));
	
	//m_mSVDDecompX(toCv(m_mHXPrime));
	//m_mSVDDecompY(toCv(m_mHYPrime));
	
	size_t time_end = ofGetElapsedTimeMicros();
	cout << "CPU Decomposition Time Elapsed : " << time_end - time_start << endl;
	//*/

	// GPU.
	cout << "GPU SVD decomposition" << endl;
	time_start = ofGetElapsedTimeMicros();
	
	gpuSVD(toCv(m_mHXPrime), &m_mSVDDecompX);
	gpuSVD(toCv(m_mHYPrime), &m_mSVDDecompY);
	
	time_end = ofGetElapsedTimeMicros();	
	cout << "GPU Decomposition Time Elapsed : " << time_end - time_start << endl;
	
}

// Computes least squares solution matrix for any right hand side b.
// Should work for square matrices.
void RigidMeshDeformer2D::gpuSVD(cv::Mat wmatrix, cv::SVD * output)
{

	/* Declare all the necessary variables */

	/* Dimensions of matrices */
	int rows    = wmatrix.rows;
	int columns = wmatrix.cols;
	int len = rows*columns;

	int M = rows;
    int N = columns;

	culaStatus status;
    
    /* Setup SVD Parameters */
    int LDA = M;
    int LDU = M;
    int LDVT = N;
   
	/*
    float* A = NULL;
    float* S = NULL;
    float* U = NULL;
    float* VT = NULL;
	*/

    char jobu  = 'A';
    char jobvt = 'A';

	// Create matrices.
	
	/*
    A = (float*)malloc(M*N*sizeof(float));
    S = (float*)malloc(imin(M,N)*sizeof(float));
    U = (float*)malloc(LDU*M*sizeof(float));
    VT = (float*)malloc(LDVT*N*sizeof(float));
	*/
	
	/* Memory allocation failed */
	
    if(!A || !S || !U || !VT) 
    {
        free(A);
        free(U);
        free(S);
        free(VT);

        return;
    }
	

	// Populate the initial matrix with the input matrix data.
	for(int r = 0; r < rows; r++)
	for(int c = 0; c < columns; c++)
	{
		int index = c*columns + r;

		culaFloat temp = wmatrix.at<double>(r, c);
		A[index] = temp;
	}

	/* Initialize CULA */
   
	/*
	status = culaInitialize();
    checkStatus(status);
	*/
		
	size_t time_start, time_end;

	time_start = ofGetElapsedTimeMicros();

    /* Perform singular value decomposition CULA */
    status = culaSgesvd(jobu, jobvt, M, N, A, LDA, S, U, LDU, VT, LDVT);
    checkStatus(status);

	time_end = ofGetElapsedTimeMicros();
	cout << "GPU_TIME = " << time_end - time_start << endl;
	    
	// Shut douwn Cula.
    //culaShutdown();

	// Double prescision resizing.
	output -> u.create(rows, rows, CV_64F);
	output -> w.create(1, columns, CV_64F);
	output -> vt.create(columns, columns, CV_64F);

	// Populate the openCV output structure.
	
	copyMatSpecial(U, &(output -> u));
	copyMatSpecial(VT, &(output -> vt));
	copyMatSpecial(S, &(output -> w));
	
    /* Free the memory. */
	
	/*
	free(A);
    free(U);
    free(S);
    free(VT);
	*/

	return;
}

// Requires a float array that will be transposed and mapped to the given destenation matrix.
inline void RigidMeshDeformer2D::copyMatSpecial(float * A, cv::Mat * dest)
{
	
	int rows = dest -> rows;
	int cols = dest -> cols;

	for(int r = 0; r < rows; r++)
	for(int c = 0; c < cols; c++)
	{
		int i1 = c*rows + r;
		dest -> at<double>(r, c) = A[i1];
	}
}

void RigidMeshDeformer2D::printMat(cv::Mat input)
{
	printf("Printing matrix\n");
	int rows = input.rows;
	int cols = input.cols;

	for(int r = 0; r < rows; r++)
	for(int c = 0; c < cols; c++)
	{
		printf("Row = %d, Col = %d, val = %f\n", r, c, input.at<double>(r, c));
	}

	printf("\n");

}

/* Check for errors and exit if one occurred */
void RigidMeshDeformer2D::checkStatus(culaStatus status)
{
    char buf[256];

    if(!status)
        return;

    culaGetErrorInfoString(status, culaGetErrorInfo(), buf, sizeof(buf));
    printf("%s\n", buf);

    culaShutdown();
    exit(EXIT_FAILURE);
}

void RigidMeshDeformer2D::ApplyFittingStep()
{
		
	// put constraints into vector (will be useful)
	std::vector<Constraint> vConstraintsVec;
	std::set<Constraint>::iterator cur(m_vConstraints.begin()), end(m_vConstraints.end());
	while ( cur != end )
		vConstraintsVec.push_back( *cur++ );
	
	unsigned int nVerts = (unsigned int)m_vDeformedVerts.size();
	size_t nConstraints = vConstraintsVec.size();
	unsigned int nFreeVerts = nVerts - nConstraints;
	
	// make vector of deformed vertex weights
	Wml::GVectord vFX( nVerts );
	Wml::GVectord vFY( nVerts );
	for ( int i = 0; i < (int)nVerts; ++i )
		vFX[i] = vFY[i] = 0.0;
	
	size_t nTriangles = m_vTriangles.size();
	for ( unsigned int i = 0; i < nTriangles; ++i ) {
		Triangle & t = m_vTriangles[i];
		
		for ( int j = 0; j < 3; ++j ) {
			
			int nA = m_vVertexMap[ t.nVerts[j] ];
			int nB = m_vVertexMap[ t.nVerts[(j+1)%3] ];
			
			ofVec2f vDeformedA;
			vDeformedA.x= t.vScaled[j].x;
			vDeformedA.y= t.vScaled[j].y;
		
			ofVec2f vDeformedB;
			vDeformedB.x= t.vScaled[(j+1)%3].x;
			vDeformedB.y= t.vScaled[(j+1)%3].y;
			
			// X elems
			vFX[nA] += -2 * vDeformedA.x+ 2 * vDeformedB.x;
			vFX[nB] += 2 * vDeformedA.x- 2 * vDeformedB.x;
			
			//  Y elems
			vFY[nA] += -2 * vDeformedA.y+ 2 * vDeformedB.y;
			vFY[nB] += 2 * vDeformedA.y- 2 * vDeformedB.y;
		}
	}
	
	
	// make F0 vectors
	Wml::GVectord vF0X( nFreeVerts ), vF0Y( nFreeVerts );
	for ( int i = 0; i < (int)nFreeVerts; ++i ) {
		vF0X[i] = vFX[i];
		vF0Y[i] = vFY[i];
	}
	
	// make Q vectors (vectors of constraints)
	Wml::GVectord vQX( (int)nConstraints ),  vQY( (int)nConstraints );
	for ( int i = 0; i < (int)nConstraints; ++i ) {
		vQX[i] = vConstraintsVec[i].vConstrainedPos.x;
		vQY[i] = vConstraintsVec[i].vConstrainedPos.y;
	}
	
	// ok, compute RHS for X and solve
	Wml::GVectord vRHSX( m_mDX * vQX );
	vRHSX += vF0X;
	vRHSX *= -1;
	Wml::GVectord vSolutionX( (int)nFreeVerts );
	
	//Wml::LinearSystemd::Solve( m_mHXPrime, vRHSX, vSolutionX );
	/*
	bool bResult = Wml::LinearSystemExtd::LUBackSub( m_mLUDecompX, vRHSX, vSolutionX );
	if (!bResult)
		DebugBreak();
	 */
	cv::Mat vRHSXMat = toCv(vRHSX), vSolutionXMat= toCv(vSolutionX);
	m_mSVDDecompX.backSubst(vRHSXMat, vSolutionXMat);

	// Bryce Line.
	//vSolutionXMat = toCv(m_GPU_SVD_X * vRHSX);
	
	// now for Y
	Wml::GVectord vRHSY( m_mDY * vQY );
	vRHSY += vF0Y;
	vRHSY *= -1;
	Wml::GVectord vSolutionY( (int)nFreeVerts );
	//	Wml::LinearSystemd::Solve( m_mHYPrime, vRHSY, vSolutionY );
	/*
	 bResult = Wml::LinearSystemExtd::LUBackSub( m_mLUDecompY, vRHSY, vSolutionY );
	if (!bResult)
		DebugBreak();
	 */
	cv::Mat vRHSYMat = toCv(vRHSY), vSolutionYMat = toCv(vSolutionY);
	m_mSVDDecompY.backSubst(vRHSYMat, vSolutionYMat);

	// Bryce Line.
	//vSolutionYMat = toCv(m_GPU_SVD_Y * vRHSY);
	
	// done!
	for ( unsigned int i = 0; i < nVerts; ++i ) {
		Constraint c(i,ofVec2f(0,0));
		if ( m_vConstraints.find(c) != m_vConstraints.end() )
			continue;
		int nRow = m_vVertexMap[i];
        
        
		m_vDeformedVerts[i].vPosition.x= (float)vSolutionX[nRow];
		m_vDeformedVerts[i].vPosition.y= (float)vSolutionY[nRow];
	}
	
}












void RigidMeshDeformer2D::PrecomputeOrientationMatrix()
{
	// put constraints into vector (will be useful)
	std::vector<Constraint> vConstraintsVec;
	std::set<Constraint>::iterator cur(m_vConstraints.begin()), end(m_vConstraints.end());
	while ( cur != end )
		vConstraintsVec.push_back( *cur++ );
	
	// resize matrix and clear to zero
	unsigned int nVerts = (unsigned int)m_vDeformedVerts.size();
	m_mFirstMatrix.SetSize( 2*nVerts, 2*nVerts);
	for ( unsigned int i = 0; i < 2*nVerts; ++i )
		for ( unsigned int j = 0; j < 2*nVerts; ++j )
			m_mFirstMatrix(i,j) = 0.0;
	
	size_t nConstraints = vConstraintsVec.size();
	unsigned int nFreeVerts = nVerts - nConstraints;
	
	// figure out vertex ordering. first do free vertices, then constraints
	unsigned int nRow = 0;
	m_vVertexMap.resize(nVerts);
	for ( unsigned int i = 0; i < nVerts; ++i ) {
		Constraint c(i, ofVec2f(0,0));
		if ( m_vConstraints.find(c) != m_vConstraints.end() )
			continue;
		m_vVertexMap[i] = nRow++;
	}
	if ( nRow != nFreeVerts )	Debugbreak();
	for ( unsigned int i = 0 ; i < nConstraints; ++i )
		m_vVertexMap[vConstraintsVec[i].nVertex ] = nRow++;
	if ( nRow != nVerts )	Debugbreak();		// bad!
	
	
	// test vector...
	Wml::GVectord gUTest( nVerts * 2 );
	for ( unsigned int i = 0; i < nVerts; ++i ) {
		Constraint c(i,ofVec2f(0,0));
		if ( m_vConstraints.find(c) != m_vConstraints.end() )
			continue;
		int nRow = m_vVertexMap[i];
		gUTest[2*nRow] = m_vInitialVerts[i].vPosition.x;
		gUTest[2*nRow+1] = m_vInitialVerts[i].vPosition.y;
	}
	for ( unsigned int i = 0; i < nConstraints; ++i ) {
		int nRow = m_vVertexMap[ vConstraintsVec[i].nVertex ];
		gUTest[2*nRow] = vConstraintsVec[i].vConstrainedPos.x;
		gUTest[2*nRow+1] = vConstraintsVec[i].vConstrainedPos.y;
	}
	
	
	// ok, now fill matrix (?)
	size_t nTriangles = m_vTriangles.size();
	for ( unsigned int i = 0; i < nTriangles; ++i ) {
		Triangle & t = m_vTriangles[i];
		
		//printf("Triangle %i: \n", i);
		double fTriSumErr = 0;
		for ( int j = 0; j < 3; ++j ) {
			double fTriErr = 0;
			
			int n0x = 2 * m_vVertexMap[ t.nVerts[j] ];
			int n0y = n0x + 1;
			int n1x = 2 * m_vVertexMap[ t.nVerts[(j+1)%3] ];
			int n1y = n1x + 1;
			int n2x = 2 * m_vVertexMap[ t.nVerts[(j+2)%3] ];
			int n2y = n2x + 1;
			float x = t.vTriCoords[j].x;
			float y = t.vTriCoords[j].y;
			//printf("---> x y %f %f \n", x, y);
			
			// DEBUG
			//ofVec2f v0( (float)gUTest[n0x], (float)gUTest[n0y] );
//			ofVec2f v1( (float)gUTest[n1x], (float)gUTest[n1y] );
//			ofVec2f v2( (float)gUTest[n2x], (float)gUTest[n2y] );
//			ofVec2f v01( v1 - v0 );
//			ofVec2f v01Perp( v01.y, -v01.x);
//			ofVec2f vTest( v0 + x * v01 + y * v01Perp );
//			float fDist = (vTest - v2).Dot(vTest - v2);
			//if ( fDist > 0.0001f )
			//	DebugBreak();
			//DEBUG
			
			//double dTest = 
			//(1 - 2*x + (x*x) + (y*y))*pow(gUTest[n0x],2) + (1 - 2*x + (x*x) + (y*y))*pow(gUTest[n0y],2) + 
			//	((x*x) + (y*y))*pow(gUTest[n1x],2) + ((x*x) + (y*y))*pow(gUTest[n1y],2) + 
			//	pow(gUTest[n2x],2) + pow(gUTest[n2y],2) + gUTest[n1y]*(-2*y*gUTest[n2x] - 2*x*gUTest[n2y]) + 
			//	gUTest[n0y]*(-2*y*gUTest[n1x] + (2*x - 2*(x*x) - 2*(y*y))*gUTest[n1y] + 2*y*gUTest[n2x] + 
			//	(-2 + 2*x)*gUTest[n2y]) + gUTest[n0x]*
			//	((2*x - 2*(x*x) - 2*(y*y))*gUTest[n1x] + 2*y*gUTest[n1y] + (-2 + 2*x)*gUTest[n2x] - 
			//	2*y*gUTest[n2y]) + gUTest[n1x]*(-2*x*gUTest[n2x] + 2*y*gUTest[n2y]);
			//_RMSInfo("TEST IS %f %f\n", dTest, fDist);
			
			// n0x,n?? elems
			m_mFirstMatrix[n0x][n0x] += 1 - 2*x + x*x + y*y;
			m_mFirstMatrix[n0x][n1x] += 2*x - 2*x*x - 2*y*y;		//m_mFirstMatrix[n1x][n0x] += 2*x - 2*x*x - 2*y*y;
			m_mFirstMatrix[n0x][n1y] += 2*y;						//m_mFirstMatrix[n1y][n0x] += 2*y;
			m_mFirstMatrix[n0x][n2x] += -2 + 2*x;					//m_mFirstMatrix[n2x][n0x] += -2 + 2*x;
			m_mFirstMatrix[n0x][n2y] += -2 * y;						//m_mFirstMatrix[n2y][n0x] += -2 * y;
			
			fTriErr += (1 - 2*x + x*x + y*y)  * gUTest[n0x] * gUTest[n0x];
			fTriErr += (2*x - 2*x*x - 2*y*y)  * gUTest[n0x] * gUTest[n1x];
			fTriErr += (2*y)                  * gUTest[n0x] * gUTest[n1y];
			fTriErr += (-2 + 2*x )            * gUTest[n0x] * gUTest[n2x];
			fTriErr += (-2 * y)               * gUTest[n0x] * gUTest[n2y];
			
			// n0y,n?? elems
			m_mFirstMatrix[n0y][n0y] += 1 - 2*x + x*x + y*y;
			m_mFirstMatrix[n0y][n1x] += -2*y;						//m_mFirstMatrix[n1x][n0y] += -2*y;
			m_mFirstMatrix[n0y][n1y] += 2*x - 2*x*x - 2*y*y;		//m_mFirstMatrix[n1y][n0y] += 2*x - 2*x*x - 2*y*y;
			m_mFirstMatrix[n0y][n2x] += 2*y;						//m_mFirstMatrix[n2x][n0y] += 2*y;
			m_mFirstMatrix[n0y][n2y] += -2 + 2*x;					//m_mFirstMatrix[n2y][n0y] += -2 + 2*x;
			
			fTriErr += (1 - 2*x + x*x + y*y)   * gUTest[n0y] * gUTest[n0y];
			fTriErr += (-2*y)                  * gUTest[n0y] * gUTest[n1x];
			fTriErr += (2*x - 2*x*x - 2*y*y)   * gUTest[n0y] * gUTest[n1y];
			fTriErr += (2*y)                   * gUTest[n0y] * gUTest[n2x];
			fTriErr += (-2 + 2*x)              * gUTest[n0y] * gUTest[n2y];
			
			// n1x,n?? elems
			m_mFirstMatrix[n1x][n1x] += x*x + y*y;
			m_mFirstMatrix[n1x][n2x] += -2*x;						//m_mFirstMatrix[n2x][n1x] += -2*x;
			m_mFirstMatrix[n1x][n2y] += 2*y;						//m_mFirstMatrix[n2y][n1x] += 2*y;
			
			fTriErr += (x*x + y*y)            * gUTest[n1x] * gUTest[n1x];
			fTriErr += (-2*x)                 * gUTest[n1x] * gUTest[n2x];
			fTriErr += (2*y)                  * gUTest[n1x] * gUTest[n2y];
			
			//n1y,n?? elems
			m_mFirstMatrix[n1y][n1y] += x*x + y*y;
			m_mFirstMatrix[n1y][n2x] += -2*y;						//m_mFirstMatrix[n2x][n1y] += -2*y;
			m_mFirstMatrix[n1y][n2y] += -2*x;						//m_mFirstMatrix[n2y][n1y] += -2*x;
			
			
			fTriErr += (x*x + y*y)            * gUTest[n1y] * gUTest[n1y];
			fTriErr += (-2*y)                 * gUTest[n1y] * gUTest[n2x];
			fTriErr += (-2*x)                 * gUTest[n1y] * gUTest[n2y];
			
			// final 2 elems
			m_mFirstMatrix[n2x][n2x] += 1;
			m_mFirstMatrix[n2y][n2y] += 1;
			
			fTriErr += gUTest[n2x] * gUTest[n2x]  +  gUTest[n2y] * gUTest[n2y] ;
			
			//_RMSInfo("  Error for vert %d (%d) - %f\n", j, t.nVerts[j], fTriErr);
			fTriSumErr += fTriErr;
		}
		//_RMSInfo("  Total Error: %f\n", fTriSumErr);
	}
	
	
	// test...
	Wml::GVectord gUTemp = m_mFirstMatrix * gUTest;
	double fSum = gUTemp.Dot( gUTest );
	//printf("    (test) Residual is %f\n", fSum);
	
	//std::exit(0);
	
	// // just try printing out matrix...
//	 for ( unsigned int i = 0; i < 2*nFreeVerts; ++i ) {
//	 for ( unsigned int j = 0 ; j < 2*nFreeVerts; ++j )
//	 printf("%5.2f ", m_mFirstMatrix(i,j));
//	 printf("| ");
//	 for ( unsigned int j = 0; j < 2*nConstraints; ++j )
//	 printf("%5.2f ", m_mFirstMatrix(i, 2*nFreeVerts+j));
//	 printf("\n");
//	 }
//	 printf("-------\n");
//	 for ( unsigned int i = 0; i < 2*nConstraints; ++i ) {
//	 for ( unsigned int j = 0 ; j < 2*nFreeVerts; ++j )
//	 printf("%5.2f ", m_mFirstMatrix(i+2*nFreeVerts,j));
//	 printf("| ");
//	 for ( unsigned int j = 0; j < 2*nConstraints; ++j )
//	 printf("%5.2f ", m_mFirstMatrix(i+2*nFreeVerts, 2*nFreeVerts+j));
//	 printf("\n");
//	 }
//	 printf("\n\n");
	 
	
	// extract G00 matrix
	Wml::GMatrixd mG00( 2*nFreeVerts, 2*nFreeVerts );
	ExtractSubMatrix( m_mFirstMatrix, 0, 0, mG00 );
	
	// extract G01 and G10 matrices
	Wml::GMatrixd mG01( 2 * (int)nFreeVerts, 2 * (int)nConstraints );
	ExtractSubMatrix( m_mFirstMatrix, 0, 2*nFreeVerts, mG01 );
	Wml::GMatrixd mG10( 2 * (int)nConstraints, 2 * (int)nFreeVerts );
	ExtractSubMatrix( m_mFirstMatrix, 2*nFreeVerts, 0, mG10 );
	
	// ok, now compute GPrime = G00 + Transpose(G00) and B = G01 + Transpose(G10)
	Wml::GMatrixd mGPrime = mG00 + mG00.Transpose();
	Wml::GMatrixd mB = mG01 + mG10.Transpose();
	
	// ok, now invert GPrime
	Wml::GMatrixd mGPrimeInverse( mGPrime.GetRows(), mGPrime.GetColumns() );
	
	/*
	 bool bInverted = Wml::LinearSystemd::Inverse( mGPrime, mGPrimeInverse );
	if (!bInverted)
		DebugBreak();
	*/
	invert(mGPrime, mGPrimeInverse);
	
	// now compute -GPrimeInverse * B
	Wml::GMatrixd mFinal = mGPrimeInverse * mB;
	mFinal *= -1;
	
	m_mFirstMatrix = mFinal;		// [RMS: not efficient!]
}

static RigidMeshDeformer2D::Triangle * g_pCurTriangle = NULL;
float g_fErrSum = 0;
void AccumErrorSum( int nRow, int nCol, float fAccum )
{
	ofVec2f & vRowVtx = g_pCurTriangle->vScaled[nRow/2];
	ofVec2f & vColVtx = g_pCurTriangle->vScaled[nCol/2];
	g_fErrSum += fAccum * vRowVtx[nRow%2] * vColVtx[nCol%2];
}

void AccumScaleEntry( Wml::GMatrixd & mF, int nRow, int nCol, double fAccum )
{
	if ( nRow < 4 && nCol < 4 ) {
		mF[nRow][nCol] += fAccum;
	} else {
		Debugbreak();
	}
}





void RigidMeshDeformer2D::PrecomputeScalingMatrices( unsigned int nTriangle )
{
	// ok now fill matrix
	Triangle & t = m_vTriangles[nTriangle];
	
	// create matrices and clear to zero
	t.mF = Wml::GMatrixd(4,4);
	t.mC = Wml::GMatrixd(4,6);
	
	// precompute coeffs
	double x01 = t.vTriCoords[0].x;
	double y01 = t.vTriCoords[0].y;
	double x12 = t.vTriCoords[1].x;
	double y12 = t.vTriCoords[1].y;
	double x20 = t.vTriCoords[2].x;
	double y20 = t.vTriCoords[2].y;
	
	double k1 = x12*y01 + (-1 + x01)*y12;
	double k2 = -x12 + x01*x12 - y01*y12;
	double k3 = -y01 + x20*y01 + x01*y20;
	double k4 = -y01 + x01*y01 + x01*y20;
	double k5 = -x01 + x01*x20 - y01*y20 ;
	
	double a = -1 + x01;
	double a1 = pow(-1 + x01,2) + pow(y01,2);
	double a2 = pow(x01,2) + pow(y01,2);
	double b =  -1 + x20;
	double b1 = pow(-1 + x20,2) + pow(y20,2);
	double c2 = pow(x12,2) + pow(y12,2);
	
	double r1 = 1 + 2*a*x12 + a1*pow(x12,2) - 2*y01*y12 + a1*pow(y12,2);
	double r2 = -(b*x01) - b1*pow(x01,2) + y01*(-(b1*y01) + y20);
	double r3 = -(a*x12) - a1*pow(x12,2) + y12*(y01 - a1*y12);
	double r5 = a*x01 + pow(y01,2);
	double r6 = -(b*y01) - x01*y20;
	double r7 = 1 + 2*b*x01 + b1*pow(x01,2) + b1*pow(y01,2) - 2*y01*y20;
	
	//  set up F matrix
	
	// row 0 mF
	t.mF[0][0] = 2*a1 + 2*a1*c2 + 2*r7;
	t.mF[0][1] = 0;
	t.mF[0][2] = 2*r2 + 2*r3 - 2*r5;
	t.mF[0][3] = 2*k1 + 2*r6 + 2*y01;
	
	// row 1
	t.mF[1][0] = 0;
	t.mF[1][1] = 2*a1 + 2*a1*c2 + 2*r7;
	t.mF[1][2] = -2*k1 + 2*k3 - 2*y01;
	t.mF[1][3] = 2*r2 + 2*r3 - 2*r5;
	
	// row 2
	t.mF[2][0] = 2*r2 + 2*r3 - 2*r5;
	t.mF[2][1] = -2*k1 + 2*k3 - 2*y01;
	t.mF[2][2] = 2*a2 + 2*a2*b1 + 2*r1;
	t.mF[2][3] = 0;
	
	//row 3
	t.mF[3][0] = 2*k1 - 2*k3 + 2*y01;
	t.mF[3][1] = 2*r2 + 2*r3 - 2*r5;
	t.mF[3][2] = 0;
	t.mF[3][3] = 2*a2 + 2*a2*b1 + 2*r1;
	
	// ok, now invert F
	Wml::GMatrixd mFInverse(4,4);
	bool bResult = Wml::LinearSystemd::Inverse(t.mF, mFInverse);
	
    mFInverse *= -1.0;
	if (!bResult) {
		Debugbreak();
	}
	t.mF =  mFInverse;
	
	// set up C matrix
	
	// row 0 mC
	t.mC[0][0] = 2*k2;
	t.mC[0][1] = -2*k1;
	t.mC[0][2] = 2*(-1-k5);
	t.mC[0][3] = 2*k3;
	t.mC[0][4] = 2*a;
	t.mC[0][5] = -2*y01;
	
	// row 1 mC
	t.mC[1][0] = 2*k1;
	t.mC[1][1] = 2*k2;
	t.mC[1][2] = -2*k3;
	t.mC[1][3] = 2*(-1-k5);
	t.mC[1][4] = 2*y01;
	t.mC[1][5] = 2*a;
	
	// row 2 mC
	t.mC[2][0] = 2*(-1-k2);
	t.mC[2][1] = 2*k1;
	t.mC[2][2] = 2*k5;
	t.mC[2][3] = 2*r6;
	t.mC[2][4] = -2*x01;
	t.mC[2][5] = 2*y01;
	
	// row 3 mC
	t.mC[3][0] = 2*k1;
	t.mC[3][1] = 2*(-1-k2);
	t.mC[3][2] = -2*k3;
	t.mC[3][3] = 2*k5;
	t.mC[3][4] = -2*y01;
	t.mC[3][5] = -2*x01;
	
	
}




void Scale( ofVec2f & vTriV0,
                ofVec2f & vTriV1,
               ofVec2f & vTriV2,
                float fScale )
{
    // find center of mass
    ofVec2f vCentroid;
    vCentroid = ( vTriV0 + vTriV1 + vTriV2 );
    vCentroid *= (float)1.0 / (float)3.0;
    
    // convert to vectors, scale and restore
    vTriV0 -= vCentroid;	vTriV0 *= fScale;	vTriV0 += vCentroid;
    vTriV1 -= vCentroid;	vTriV1 *= fScale;	vTriV1 += vCentroid;
    vTriV2 -= vCentroid;	vTriV2 *= fScale;	vTriV2 += vCentroid;
}


void RigidMeshDeformer2D::UpdateScaledTriangle( unsigned int nTriangle )
{
	// ok now fill matrix
	Triangle & t = m_vTriangles[nTriangle];
	
	// multiply mC by deformed vertex position
	const ofVec2f & vDeformedV0 = m_vDeformedVerts[ t.nVerts[0] ].vPosition;
	const ofVec2f & vDeformedV1 = m_vDeformedVerts[ t.nVerts[1] ].vPosition;
	const ofVec2f & vDeformedV2 = m_vDeformedVerts[ t.nVerts[2] ].vPosition;
	double tmp[6] = { vDeformedV0.x, vDeformedV0.y, 
		vDeformedV1.x, vDeformedV1.y, 
		vDeformedV2.x, vDeformedV2.y};
	Wml::GVectord vDeformed( 6, tmp );
	Wml::GVectord mCVec = t.mC * vDeformed;
	
	// compute -MFInv * mC
	Wml::GVectord vSolution = t.mF * mCVec;
	
	// ok, grab deformed v0 and v1 from solution vector
	ofVec2f vFitted0( (float)vSolution[0], (float)vSolution[1] );
	ofVec2f vFitted1( (float)vSolution[2], (float)vSolution[3] );
	
	// figure out fitted2
	float x01 = t.vTriCoords[0].x;
	float y01 = t.vTriCoords[0].y;
	ofVec2f vFitted01( vFitted1 - vFitted0 );
	ofVec2f vFitted01Perp( vFitted01.y, -vFitted01.x);
	ofVec2f vFitted2( vFitted0 + (float)x01 * vFitted01 + (float)y01 * vFitted01Perp );
	
	// ok now determine scale
	ofVec2f & vOrigV0 = m_vInitialVerts[ t.nVerts[0] ].vPosition;
	ofVec2f & vOrigV1 = m_vInitialVerts[ t.nVerts[1] ].vPosition;
	float fScale = ( vOrigV1 - vOrigV0 ).length() / vFitted01.length();
	
	// now scale triangle
	Scale( vFitted0, vFitted1, vFitted2, fScale );
	
	t.vScaled[0] = vFitted0;
	t.vScaled[1] = vFitted1;
	t.vScaled[2] = vFitted2;
}



void RigidMeshDeformer2D::ValidateDeformedMesh( bool bRigid )
{
	size_t nConstraints = m_vConstraints.size();
	if ( nConstraints < 2 )
		return;
	
	ValidateSetup();
	
    // make q vector of constraints
	Wml::GVectord vQ( 2 * (int)nConstraints );
	int k = 0;
	std::set<Constraint>::iterator cur(m_vConstraints.begin()), end(m_vConstraints.end());
	while ( cur != end ) {
		//Constraint & c = (*cur++);
		vQ[ 2*k ] = (*cur).vConstrainedPos.x;
		vQ[ 2*k + 1] = (*cur).vConstrainedPos.y;
		cur++;
		++k;
	}
	
	Wml::GVectord vU = m_mFirstMatrix * vQ;
	size_t nVerts = m_vDeformedVerts.size();
	for ( unsigned int i = 0; i < nVerts; ++i ) {
		Constraint c(i, ofVec2f(0,0));
		if ( m_vConstraints.find(c) != m_vConstraints.end() )
			continue;		
		int nRow = m_vVertexMap[i];
		
		double fX = vU[ 2*nRow ];
		double fY = vU[ 2*nRow + 1];
		m_vDeformedVerts[i].vPosition = ofVec2f( (float)fX, (float)fY );
	}
	
	if ( bRigid ) {
		// ok, now scale triangles
		size_t nTris = m_vTriangles.size();
		for ( unsigned int i = 0; i < nTris; ++i )
			UpdateScaledTriangle(i);
		
		
		ApplyFittingStep();
	}
}


