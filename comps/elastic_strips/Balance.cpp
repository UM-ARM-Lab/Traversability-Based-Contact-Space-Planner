
#include "stdafx.h"

Balance::Balance(RobotBasePtr r, std::vector<string> s_links, Vector p_scale, Vector p_trans)
{
    // RAVELOG_INFO("Balance Constructor.\n");
    _Robot = r;
    _supportlinks = s_links;
    _polyscale = p_scale;
    _polytrans = p_trans;
    _balance_mode = BALANCE_SUPPORT_POLYGON;

    for(int i = 0; i < s_links.size(); i++)
    {
        // _manip_index_name_map.insert(std::make_pair(i,s_links[i]));
		_manip_name_index_map.insert(std::make_pair(s_links[i],i));
    }

    // RAVELOG_INFO("Initializing parameters.\n");
    InitializeBalanceParameters();
}

Balance::Balance(RobotBasePtr r, Vector g, std::vector<string> s_manips, std::vector<dReal> s_mus)
{
    _Robot = r;
    _gravity = g;
    _support_manips = s_manips;
    _support_mus = s_mus;
    _balance_mode = BALANCE_GIWC;

    for(int i = 0; i < s_manips.size(); i++)
    {
        // _manip_index_name_map.insert(std::make_pair(i,s_manips[i]));
		_manip_name_index_map.insert(std::make_pair(s_manips[i],i));
    }

    // InitializeBalanceParameters();
}

NEWMAT::ReturnMatrix Balance::GetSurfaceCone(string& manipname, dReal mu)
{
    if(_computed_contact_surface_cones.count(manipname) != 0){
        return _computed_contact_surface_cones.find(manipname)->second;
    }
    else{
        RobotBase::ManipulatorPtr p_manip = _Robot->GetManipulator(manipname);
        Vector manip_dir = p_manip->GetLocalToolDirection();

        std::vector<Vector> support_points = GetSupportPoints(p_manip);
        int num_points = support_points.size();

        // Calculate combined friction cone matrix
        int rows = CONE_DISCRETIZATION_RESOLUTION*num_points;
        int cols = 3*num_points;
        NEWMAT::Matrix f_cones_diagonal(rows, cols);
        f_cones_diagonal = 0.0; // All non-filled spots should be 0

        // Fill the diagonals
        for (int i = 0; i < num_points; i++) {
            GetFrictionCone(support_points[i], manip_dir, mu, &f_cones_diagonal, i*CONE_DISCRETIZATION_RESOLUTION, i*3, p_manip->GetTransform());
        }

        // Calculate A_surf matrix
        NEWMAT::Matrix a_surf_stacked(cols, 6); // TODO: Rename `cols` because it's the rows here

        for (int i = 0; i < num_points; i++) {
            // Cone transform has no rotation relative to the manipulator's transform and is translated by
            // the vector contained in support_points
            Transform cone_tf;
            cone_tf.trans = support_points[i];
            GetASurf(p_manip, cone_tf, &a_surf_stacked, 3*i);
        }

        NEWMAT::Matrix mat = f_cones_diagonal * a_surf_stacked; // Dot product

        // omit redundant rows, useless, no redundant rows
        dd_ErrorType err;
        dd_MatrixPtr contact_span_cdd = dd_CreateMatrix(mat.Nrows(), mat.Ncols()+1);
        for (int r = 0; r < mat.Nrows(); r++) {
        // First element of each row indicates whether it's a point or ray. These are all rays, indicated by 0.
            dd_set_si(contact_span_cdd->matrix[r][0], 0.0);
            for (int c = 0; c < mat.Ncols(); c++) {
                dd_set_si(contact_span_cdd->matrix[r][c+1], mat(r+1, c+1));
            }
        }

        _computed_contact_surface_cones.insert(std::pair<string,NEWMAT::Matrix>(manipname,mat));

        dd_FreeMatrix(contact_span_cdd);
        f_cones_diagonal.ReleaseAndDelete();
        a_surf_stacked.ReleaseAndDelete();
        mat.Release();

        return mat;
    }
}

/// Returns the support points relative to the world frame
void Balance::GetSupportPointsForLink(RobotBase::LinkPtr p_link, Vector tool_dir, Transform result_tf, std::vector<Vector>& contacts) {
    Transform tf = result_tf.inverse() * p_link->GetTransform();

    AABB aabb = p_link->ComputeLocalAABB();

    // If any extent is 0, the link has no volume and is assumed to be a virtual link
    if (aabb.extents.x <= 0 || aabb.extents.y <= 0 || aabb.extents.z <= 0) {
        return;
    }

    if(strcmp(p_link->GetName().c_str(), "l_foot") != 0 &&
       strcmp(p_link->GetName().c_str(), "r_foot") != 0 &&
       strcmp(p_link->GetName().c_str(), "l_palm") != 0 &&
       strcmp(p_link->GetName().c_str(), "r_palm") != 0)
    {
        return;
    }

    // Iterates over the 8 combinations of (+ or -) for each of the 3 dimensions
    for (int neg_mask = 0; neg_mask < 8; neg_mask++) {
        Vector contact;
        bool is_invalid = false;

        // Iterate over x, y, and z (compiler probably unrolls this loop)
        for (int axis = 0; axis < 3; axis++) {
            bool neg = !!(neg_mask&(1<<axis));
            // A point will be "invalid" if it is opposite the local tool direction
            is_invalid = is_invalid || (neg && (tool_dir[axis] > 0.85)) || (!neg && (tool_dir[axis] < -0.85));
            if (is_invalid) break;

            contact[axis] = aabb.pos[axis] + (neg ? -1 : 1)*aabb.extents[axis];
        }

        if (!is_invalid) {
            Vector contact_t = tf * contact;
            contacts.push_back(contact_t);

//            return; //! TEMP
        }
    }
}

/// Returns the support points of the given manipulator in the WORLD frame
std::vector<Vector> Balance::GetSupportPoints(RobotBase::ManipulatorPtr p_manip) {
    // Get all rigidly attached links -- in my case, the end effector link is a virtual
    // link with 0 volume. The actual physical ee link is rigidly attached to it.
    std::vector<RobotBase::LinkPtr> attached_links;
    p_manip->GetEndEffector()->GetRigidlyAttachedLinks(attached_links);

    // Other manipulator info
    Transform world_to_manip = p_manip->GetTransform().inverse();
    Vector tool_dir = p_manip->GetLocalToolDirection();

    std::vector<Vector> contacts;
    for (int i = 0; i < attached_links.size(); i++) {
        const char* link_name = attached_links[i]->GetName().c_str();

        // Transforms the tool_dir into the link frame
        GetSupportPointsForLink(attached_links[i], world_to_manip * attached_links[i]->GetTransform() * tool_dir, p_manip->GetTransform(), contacts);
    }
    return contacts;
}

void Balance::GetFrictionCone(Vector& center, Vector& direction, dReal mu, NEWMAT::Matrix* mat, int offset_r, int offset_c, Transform temp_tf) {
    // This sets `a` (for axis) to the index of `direction` that is nonzero, sets `c` (cos) to the first index that
    // is zero, and `s` (sin) to the second that is zero. Formulas derived from a truth table.
    // NOTE: 1-based indexing
    int a = direction[0] ? 1 : direction[1] ? 2 : 3;
    int c = 1 + (a == 1); // 1 if `a` is not 1, 2 otherwise.
    int s = 3 - (a == 3); // 3 if `a` is not 3, 2 otherwise

    dReal step = M_PI * 2.0 / CONE_DISCRETIZATION_RESOLUTION;
    dReal angle = 0;
    // NOTE 1-based indexing
    for (int i = 1; i <= CONE_DISCRETIZATION_RESOLUTION; i++) {
        // a-colum will be -1 or 1. The -1 multiplication is because friction force occurs in the opposite direction
        (*mat)(offset_r + i, offset_c + a) = direction[a-1] * -1;
        (*mat)(offset_r + i, offset_c + s) = round(mu * sin(angle) * 10000) * 0.0001;
        (*mat)(offset_r + i, offset_c + c) = round(mu * cos(angle) * 10000) * 0.0001;
        angle += step;
    }

}

void Balance::GetASurf(RobotBase::ManipulatorPtr p_manip, Transform cone_to_manip, NEWMAT::Matrix *mat, int offset_r) {
    Transform manip_to_world = p_manip->GetTransform(); // For testing
    TransformMatrix tf_matrix = TransformMatrix(cone_to_manip); //TransformMatrix(cone_to_manip);

    // First 3 columns are just the tf_matrix
    for (int r = 0; r < 3; r++) {
        for (int c = 0; c < 3; c++) {
            (*mat)(offset_r + r + 1, c + 1) =  tf_matrix.m[ r*4 + c ];
        }
    }

    // (Notes originally written for Python)
    // To calculate v, we take the origin of the friction cone in the world frame with cone_tf.trans,
    // then transform it to the surface frame by multipling by T_W_s = T_s_W.I. This gives the displacement from
    // surface frame to contact frame. To get the displacement from contact frame to surface frame, it is multiplied
    // by -1 to reverse direction b/c disp(a, b) = -disp(b, a).
    Vector v = (cone_to_manip.trans);
    v *= -1;
    // NOTE The above math messes up the 4th element of v, which must be 1 for affine transforms
    v[3] = 1;

    // Last 3 columns are the cross-product-equivalent matrix for r
    (*mat)(offset_r + 1, 4) =  0.0;
    (*mat)(offset_r + 1, 5) = -v.z;
    (*mat)(offset_r + 1, 6) =  v.y;
    (*mat)(offset_r + 2, 4) =  v.z;
    (*mat)(offset_r + 2, 5) =  0.0;
    (*mat)(offset_r + 2, 6) = -v.x;
    (*mat)(offset_r + 3, 4) = -v.y;
    (*mat)(offset_r + 3, 5) =  v.x;
    (*mat)(offset_r + 3, 6) =  0.0;
}

void Balance::GetAStance(Transform tf, NEWMAT::Matrix* mat, int offset_r) {
    // Note: This AStance computes the transpose of the AStance from the GIWC paper,
    // because of the way the cone representation is done here
    TransformMatrix m = TransformMatrix(tf);

    // Create -R matrix
    NEWMAT::Matrix negR_T(3, 3);
    negR_T << -m.m[0] << -m.m[4] << -m.m[8]
           << -m.m[1] << -m.m[5] << -m.m[9]
           << -m.m[2] << -m.m[6] << -m.m[10];

    // Create cross-product-equivalent matrix of the transform's translation component
    NEWMAT::Matrix crossP_T(3, 3);
    crossP_T <<  0.0        <<  tf.trans.z << -tf.trans.y
             << -tf.trans.z <<  0.0        <<  tf.trans.x
             <<  tf.trans.y << -tf.trans.x <<  0.0       ;

    // Create TRANSPOSE OF matrix [    -R        0 ]
    //                            [ [p]x * -R   -R ]
    (*mat).SubMatrix(offset_r + 1, offset_r + 3, 1, 3) = negR_T;
    // Computes transpose of multiplication by using (negR * crossP)_T = negR_T * crossP_T
    (*mat).SubMatrix(offset_r + 1, offset_r + 3, 4, 6) = negR_T * crossP_T;
    (*mat).SubMatrix(offset_r + 4, offset_r + 6, 1, 3) = 0.0;
    (*mat).SubMatrix(offset_r + 4, offset_r + 6, 4, 6) = negR_T;

    negR_T.ReleaseAndDelete();
    crossP_T.ReleaseAndDelete();
}

NEWMAT::ReturnMatrix Balance::GetGIWCSpanForm()
{
    int num_manips = _support_manips.size();
    std::vector<NEWMAT::Matrix> matrices;
    int total_rows = 0;

    for (int i = 0; i < num_manips; i++) {
        matrices.push_back(GetSurfaceCone(_support_manips[i], _support_mus[i]));
        total_rows += matrices.back().Nrows();
    }

    // Calculate combined surface cone matrix
    NEWMAT::Matrix s_cones_diagonal(total_rows, 6*num_manips);
    s_cones_diagonal = 0.0;

    int current_row_offset = 0;
    for (int i = 0; i < num_manips; i++) {
        s_cones_diagonal.SubMatrix(current_row_offset+1, current_row_offset+matrices[i].Nrows(), (6*i)+1, (6*i)+6) = matrices[i];
        current_row_offset += matrices[i].Nrows();
    }

    // Calculate A_stance matrix
    NEWMAT::Matrix a_stance_stacked(6 * num_manips, 6);

    for (int i = 0; i < num_manips; i++) {
        GetAStance(_Robot->GetManipulator(_support_manips[i])->GetTransform(), &a_stance_stacked, 6*i);
    }

    NEWMAT::Matrix mat = s_cones_diagonal * a_stance_stacked; // Dot product

    s_cones_diagonal.ReleaseAndDelete();
    a_stance_stacked.ReleaseAndDelete();
    for(unsigned int i = 0; i < matrices.size(); i++)
    {
        matrices[i].ReleaseAndDelete();
    }
    mat.Release();
    return mat;
}

void Balance::GetGIWC()
{    
    dd_set_global_constants();
    dd_ErrorType err;

    NEWMAT::Matrix giwc_span = GetGIWCSpanForm();


    // dd_MatrixPtr giwc_span_cdd = dd_CreateMatrix(giwc_span.Nrows(), giwc_span.Ncols()+1);
    // giwc_span_cdd->representation = dd_Generator;

    std::vector<coordT> pointsIn((giwc_span.Nrows()+1)*6);
    pointsIn[0] = 0;
    pointsIn[1] = 0;
    pointsIn[2] = 0;
    pointsIn[3] = 0;
    pointsIn[4] = 0;
    pointsIn[5] = 0;

    // TODO: Is there a better way than doing this?
    for (int r = 0; r < giwc_span.Nrows(); r++) {
        // First element of each row indicates whether it's a point or ray. These are all rays, indicated by 0.
        // dd_set_si(giwc_span_cdd->matrix[r][0], 0);
        // dd_set_si(giwc_span_cdd->matrix[r][0], 0.000001);
        for (int c = 0; c < giwc_span.Ncols(); c++) {
            // It's legal to multiply an entire row by the same value (here 1e4)
            // This rounds everything down to a fixed precision int
            // dd_set_si(giwc_span_cdd->matrix[r][c+1], (long) (giwc_span(r+1, c+1) * CONE_COMPUTATION_PRECISION));
            // dd_set_si(giwc_span_cdd->matrix[r][c+1], round(giwc_span(r+1, c+1) * CONE_COMPUTATION_PRECISION));
            pointsIn[(r+1)*6 + c] = 0.0001 * round(giwc_span(r+1, c+1) * 10000);
        }
    }

    int numPointsIn = giwc_span.Nrows()+1;
    std::vector< std::vector<double> > facet_coefficients;
    convexHull6D(&pointsIn[0], numPointsIn, facet_coefficients);

    // dd_PolyhedraPtr poly = dd_DDMatrix2Poly2(giwc_span_cdd, dd_MaxCutoff, &err);
    // if (err != dd_NoError) {
    //     RAVELOG_INFO("CDD Error: ");
    //     dd_WriteErrorMessages(stdout, err);
    //     throw OPENRAVE_EXCEPTION_FORMAT("CDD Error: %d", err, ORE_InvalidState);
    // }

    // dd_MatrixPtr giwc_face_cdd = dd_CopyInequalities(poly);

    // _giwc.ReSize(giwc_face_cdd->rowsize, 6);

    // for (int row = 1; row <= giwc_face_cdd->rowsize; row++) {
    //     // Note this skips element 0 of each row, which should always be 0
    //     for (int col = 2; col <= giwc_face_cdd->colsize; col++) {
    //         _giwc(row, col-1) = dd_get_d(giwc_face_cdd->matrix[row-1][col-1]);
    //     }
    // }

    _giwc.ReSize(facet_coefficients.size(), 6);

    for (int row = 0; row < facet_coefficients.size(); row++) {
        // Note this skips element 0 of each row, which should always be 0
        for (int col = 1; col < 7; col++) {
            _giwc(row+1, col) = -facet_coefficients[row][col];
        }
    }

    giwc_span.ReleaseAndDelete();
    // dd_FreeMatrix(giwc_face_cdd);
    // dd_FreePolyhedra(poly);
    // dd_FreeMatrix(giwc_span_cdd);
    dd_free_global_constants();

}

int Balance::convexHull2D(coordT* pointsIn, int numPointsIn, coordT** pointsOut, int* numPointsOut)
{

    char flags[250];
    int exitcode;
    facetT *facet, *newFacet;
    int curlong, totlong;
    vertexT *vertex, *vertexA, *vertexB;
    int j;


    sprintf (flags, "qhull QJ Pp s Tc ");
    //FILE* junk = fopen("qhullout.txt","w");

    exitcode= qh_new_qhull (2, numPointsIn, pointsIn, false,
                            flags, NULL, stderr);
    //fclose(junk);
    *numPointsOut = qh num_vertices;
    *pointsOut = (coordT *)malloc(sizeof(coordT)*(*numPointsOut)*2);

    FORALLfacets {
        facet->seen = 0;
    }

    FORALLvertices {
        vertex->seen = 0;
    }

    facet=qh facet_list;
    j=0;

    while(1) {
        if (facet==NULL) {
            // empty hull
            break;
        }
        vertexA = (vertexT*)facet->vertices->e[0].p;
        vertexB = (vertexT*)facet->vertices->e[1].p;
        if (vertexA->seen==0) {
            vertexA->seen = 1;
            (*pointsOut)[j++] = vertexA->point[0];
            (*pointsOut)[j++] = vertexA->point[1];
        }
        if (vertexB->seen==0) {
            vertexB->seen = 1;
            (*pointsOut)[j++] = vertexB->point[0];
            (*pointsOut)[j++] = vertexB->point[1];
        }


        //qh_printfacet(stderr, facet);
        facet->seen = 1;
        newFacet = (facetT*)facet->neighbors->e[0].p;
        if (newFacet->seen==1) newFacet = (facetT*)facet->neighbors->e[1].p;
        if (newFacet->seen==1) { break; }
        facet = newFacet;
    }

    qh_freeqhull(!qh_ALL);
    qh_memfreeshort (&curlong, &totlong);

    return exitcode;
}

int Balance::convexHull6D(coordT* pointsIn, int numPointsIn, std::vector< std::vector<double> >& facet_coefficients) {

    char flags[250];
    int exitcode;
    facetT *facet, *newFacet;
    int curlong, totlong;
    facet_coefficients.clear();

    float min_value = 0.000001;

    sprintf (flags, "qhull QJ Pp s Tc ");

    // cout<<"input points."<<endl;
    // for(int i = 0; i < numPointsIn; i++)
    // {
    //     cout<<pointsIn[i*6+0]<<" "<<pointsIn[i*6+1]<<" "<<pointsIn[i*6+2]<<" "<<pointsIn[i*6+3]<<" "<<pointsIn[i*6+4]<<" "<<pointsIn[i*6+5]<<endl;
    // }

    exitcode= qh_new_qhull (6, numPointsIn, pointsIn, false, flags, NULL, stderr);

    FORALLfacets {
        facet->seen = 0;
    }

    facet = qh facet_list;
    int facet_numbers = qh num_facets;

    set<string> facet_hash_set;
    stringstream coeff_string;
    int rounded_coeff;

    // cout<<"facet parameters"<<endl;

    for(int i = 0; i < facet_numbers; i++)
    {
        // get the facet equations.
        coordT* normal = facet->normal;
        coordT offset = facet->offset;
        
        string hash = "";

        if(fabs(offset) < 0.0001)
        {
            std::vector<double> coeff(7);
            if(fabs(offset) < min_value)
            {
                coeff[0] = 0;
            }
            else
            {
                coeff[0] = offset;
            }
            rounded_coeff = round(coeff[0]*1000);
            if(rounded_coeff == 0) rounded_coeff = 0; // eliminate negative 0
            coeff_string << rounded_coeff;
            hash = hash + coeff_string.str();
            coeff_string.str(std::string());
            coeff_string.clear();

            for(int i = 0; i < 6; i++)
            {
                if(fabs(normal[i]) < min_value)
                {
                    coeff[i+1] = 0;
                }
                else
                {
                    coeff[i+1] = normal[i];
                }
                rounded_coeff = round(coeff[i+1]*1000);
                if(rounded_coeff == 0) rounded_coeff = 0; // eliminate negative 0
                coeff_string << rounded_coeff;
                hash = hash + "," + coeff_string.str();
                coeff_string.str(std::string());
                coeff_string.clear();
            }

            if(facet_hash_set.find(hash) == facet_hash_set.end())
            {
                facet_coefficients.push_back(coeff);
                facet_hash_set.insert(hash);
                // cout<<hash<<endl;
                // cout<<coeff[0]<<" "<<coeff[1]<<" "<<coeff[2]<<" "<<coeff[3]<<" "<<coeff[4]<<" "<<coeff[5]<<" "<<coeff[6]<<endl;
            }

        }
        if(i != facet_numbers-1)
        {
            newFacet = (facetT*)facet->next;
            facet = newFacet;
        }

    }

    qh_freeqhull(!qh_ALL);
    qh_memfreeshort (&curlong, &totlong);

    return exitcode;
}


Balance::Point2D Balance::compute2DPolygonCentroid(const Balance::Point2D* vertices, int vertexCount)
{
    Point2D centroid = {0, 0};
    double signedArea = 0.0;
    double x0 = 0.0; // Current vertex X
    double y0 = 0.0; // Current vertex Y
    double x1 = 0.0; // Next vertex X
    double y1 = 0.0; // Next vertex Y
    double a = 0.0;  // Partial signed area

    // For all vertices except last
    int i=0;
    for (i=0; i<vertexCount-1; ++i)
    {
        x0 = vertices[i].x;
        y0 = vertices[i].y;
        x1 = vertices[i+1].x;
        y1 = vertices[i+1].y;
        a = x0*y1 - x1*y0;
        signedArea += a;
        centroid.x += (x0 + x1)*a;
        centroid.y += (y0 + y1)*a;
    }

    // Do last vertex
    x0 = vertices[i].x;
    y0 = vertices[i].y;
    x1 = vertices[0].x;
    y1 = vertices[0].y;
    a = x0*y1 - x1*y0;
    signedArea += a;
    centroid.x += (x0 + x1)*a;
    centroid.y += (y0 + y1)*a;

    signedArea *= 0.5;
    centroid.x /= (6.0*signedArea);
    centroid.y /= (6.0*signedArea);

    return centroid;
}

void Balance::GetSupportPolygon()
{
    // RAVELOG_INFO("GetSupportPolygon");
    int numsupportlinks = _supportlinks.size();

    std::vector<boost::shared_ptr<KinBody::Link::Geometry> >_listGeomProperties;
    std::vector<Vector> points;
    AABB bounds;
    //get points on trimeshes of support links
    vector<KinBody::LinkPtr> vlinks = _Robot->GetLinks();
    for(int i = 0 ; i < numsupportlinks; i++)
    {
        for(int j =0; j < vlinks.size(); j++)
        {
            if(strcmp(_supportlinks[i].c_str(), vlinks[j]->GetName().c_str()) == 0 )
            {
                RAVELOG_DEBUG("Found match!\n");
                _listGeomProperties = vlinks[j]->GetGeometries();

                //compute AABBs for the link at identity
                for(int k = 0; k < _listGeomProperties.size(); k++)
                {
                    //if( _listGeomProperties.size() == 1){
                    Transform _t = _listGeomProperties[k]->GetTransform().inverse();
                    //bounds = _listGeomProperties[k]->ComputeAABB(_t);
                    bounds = _listGeomProperties[k]->ComputeAABB(_listGeomProperties[k]->GetTransform());
                    Transform offset = vlinks[j]->GetTransform()*_t;
                    points.push_back(offset*Vector(bounds.pos.x + bounds.extents.x,bounds.pos.y + bounds.extents.y,bounds.pos.z - bounds.extents.z));
                    points.push_back(offset*Vector(bounds.pos.x - bounds.extents.x,bounds.pos.y + bounds.extents.y,bounds.pos.z - bounds.extents.z));
                    points.push_back(offset*Vector(bounds.pos.x - bounds.extents.x,bounds.pos.y - bounds.extents.y,bounds.pos.z - bounds.extents.z));
                    points.push_back(offset*Vector(bounds.pos.x + bounds.extents.x,bounds.pos.y - bounds.extents.y,bounds.pos.z - bounds.extents.z));
                    
                    points.push_back(offset*Vector(bounds.pos.x + bounds.extents.x,bounds.pos.y + bounds.extents.y,bounds.pos.z + bounds.extents.z));
                    points.push_back(offset*Vector(bounds.pos.x - bounds.extents.x,bounds.pos.y + bounds.extents.y,bounds.pos.z + bounds.extents.z));
                    points.push_back(offset*Vector(bounds.pos.x - bounds.extents.x,bounds.pos.y - bounds.extents.y,bounds.pos.z + bounds.extents.z));
                    points.push_back(offset*Vector(bounds.pos.x + bounds.extents.x,bounds.pos.y - bounds.extents.y,bounds.pos.z + bounds.extents.z));
                }
                break;
            }
        }
    }
    // RAVELOG_INFO("Num support points in to qhull: %d\n",points.size());
    std::vector<coordT> pointsin(points.size()*2);
    std::vector<RaveVector<float> > plotvecs(points.size());
    for(int i = 0; i < points.size();i++)
    {
        pointsin[i*2 + 0] = points[i].x;
        pointsin[i*2 + 1] = points[i].y;
        plotvecs[i] = RaveVector<float>(points[i].x,points[i].y,points[i].z);
    }

    coordT* pointsOut = NULL;

    int numPointsOut = 0;

    convexHull2D(&pointsin[0], points.size(), &pointsOut, &numPointsOut);
    // RAVELOG_INFO("Num support points out of qhull:: %d\n",numPointsOut);
    
    std::vector<RaveVector<float> > tempvecs(numPointsOut +1);
    _supportpolyx.resize(numPointsOut);
    _supportpolyy.resize(numPointsOut);

    Point2D polygon[numPointsOut];
    dReal centerx = 0;
    dReal centery = 0;
    for(int i =0; i < numPointsOut; i++)
    {
        _supportpolyx[i] = pointsOut[(i)*2 + 0];
        _supportpolyy[i] = pointsOut[(i)*2 + 1];
        polygon[i].x = _supportpolyx[i];
        polygon[i].y = _supportpolyy[i];
        //centerx += supportpolyx[i];
        //centery += supportpolyy[i];
    }

    size_t vertexCount = sizeof(polygon) / sizeof(polygon[0]);
    Point2D centroid = compute2DPolygonCentroid(polygon, vertexCount);
    //std::cout << "Centroid is (" << centroid.x << ", " << centroid.y << ")\n";


    centerx = centroid.x;//centerx/numPointsOut;
    centery = centroid.y;//centery/numPointsOut;
    //RAVELOG_INFO("center %f %f\n",centerx, centery);

    for(int i =0; i < numPointsOut; i++)
    {
        _supportpolyx[i] = _polyscale.x*(_supportpolyx[i] - centerx) + centerx + _polytrans.x;
        _supportpolyy[i] = _polyscale.y*(_supportpolyy[i] - centery) + centery + _polytrans.y;
        tempvecs[i] = RaveVector<float>(_supportpolyx[i],_supportpolyy[i],0);
    }


    //close the polygon
    tempvecs[tempvecs.size()-1] = RaveVector<float>(_supportpolyx[0],_supportpolyy[0],0);
    // GraphHandlePtr graphptr = GetEnv()->drawlinestrip(&tempvecs[0].x,tempvecs.size(),sizeof(tempvecs[0]),5, RaveVector<float>(0, 1, 1, 1));
    // graphptrs.push_back(graphptr);


    free(pointsOut);
}

void Balance::RefreshBalanceParameters(std::vector<dReal> q_new)
{
    _Robot->SetActiveDOFValues(q_new);
    
    if(_balance_mode == BALANCE_SUPPORT_POLYGON)
    {
        GetSupportPolygon();
    }
    else if(_balance_mode == BALANCE_GIWC)
    {
        GetGIWC();
    }

}

void Balance::RefreshBalanceParameters(std::vector<dReal> q_new, std::vector<string> s_links_manips, std::map<string,NEWMAT::Matrix>& giwc_database)
{

    _Robot->SetActiveDOFValues(q_new);

    if(_balance_mode == BALANCE_SUPPORT_POLYGON)
    {
        _supportlinks = s_links_manips;
        GetSupportPolygon();
    }
    else if(_balance_mode == BALANCE_GIWC)
    {
        string pose_code;
        stringstream p;

        for(int i = 0; i < _support_manips.size(); i++)
        {
            string manip_name = _support_manips[i];

            p << manip_name << ",";

            Transform manip_transform = _Robot->GetManipulator(manip_name)->GetTransform();

            p << int(round(manip_transform.trans.x*100)) << "," << int(round(manip_transform.trans.y*100)) << "," << int(round(manip_transform.trans.z*100)) << ",";

            p << int(round(manip_transform.rot.x*100)) << "," << int(round(manip_transform.rot.y*100)) << "," << int(round(manip_transform.rot.z*100)) << "," << int(round(manip_transform.rot.w*100));
        }

        pose_code = p.str();

        if(giwc_database.count(pose_code) == 0)
        {
            GetGIWC();
            giwc_database.insert(std::pair<string,NEWMAT::Matrix>(pose_code,_giwc));
        }
        else
        {
            _giwc = giwc_database.find(pose_code)->second;
        }

    }

}

void Balance::InitializeBalanceParameters()
{
    if(_balance_mode == BALANCE_SUPPORT_POLYGON)
    {
        GetSupportPolygon();
    }
    else if(_balance_mode == BALANCE_GIWC)
    {
        GetGIWC();
    }
}

bool Balance::CheckSupport(Vector center)
{
    bool balanced = false;
    if (_balance_mode == BALANCE_SUPPORT_POLYGON) {
        int nvert = _supportpolyx.size();
        if(nvert == 0)
            return false;


        dReal testx = center.x;
        dReal testy = center.y;
        dReal * vertx = &_supportpolyx[0];
        dReal * verty = &_supportpolyy[0];


        int i, j;
        for (i = 0, j = nvert-1; i < nvert; j = i++) {
        if ( ((verty[i]>testy) != (verty[j]>testy)) &&
         (testx < (vertx[j]-vertx[i]) * (testy-verty[i]) / (verty[j]-verty[i]) + vertx[i]) )
           balanced = !balanced;
        }

        center.z = 0;
    } else if (_balance_mode == BALANCE_GIWC) {
        Vector crossprod = center.cross(_gravity);

        NEWMAT::ColumnVector giwc_test_vector(6);
        giwc_test_vector << _gravity.x << _gravity.y << _gravity.z
                         << crossprod.x << crossprod.y << crossprod.z;

        NEWMAT::ColumnVector result = _giwc * giwc_test_vector;

        balanced = true;
        // Test to see if any item in the result is less than 0
        // NOTE: 1-indexed
        for (int i = 1; i <= result.Nrows(); i++) {
            if (result(i) < 0) {
                balanced = false;
                break;
            }
        }

    }

    //RAVELOG_INFO("cog: %f %f %f\n", center.x,center.y,center.z);
    if(balanced)
    {
        // if (bPRINT)
        //     RAVELOG_INFO("Balance check: Balanced \n");

        return true;
    }
    else
    {
        // if (bPRINT)
        //     RAVELOG_INFO("Balance check: Not balanced \n");

        return false;
    }
}
