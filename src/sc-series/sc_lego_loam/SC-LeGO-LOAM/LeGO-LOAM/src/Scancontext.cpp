#include "Scancontext.h"

// namespace SC2
// {



float rad2deg(float radians)
{
    return radians * 180.0 / M_PI;
}

float deg2rad(float degrees)
{
    return degrees * M_PI / 180.0;
}


float xy2theta( const float & _x, const float & _y )
{
    if ( _x >= 0 & _y >= 0) 
        return (180/M_PI) * atan(_y / _x);

    if ( _x < 0 & _y >= 0) 
        return 180 - ( (180/M_PI) * atan(_y / (-_x)) );

    if ( _x < 0 & _y < 0) 
        return 180 + ( (180/M_PI) * atan(_y / _x) );

    if ( _x >= 0 & _y < 0)
        return 360 - ( (180/M_PI) * atan((-_y) / _x) );
} //


MatrixXd circshift( MatrixXd &_mat, int _num_shift )
{// _mat:nearest neighbors   //_num_shift: current's cols'id
    // shift columns to right direction 
    assert(_num_shift >= 0);

    if( _num_shift == 0 )
    {
        MatrixXd shifted_mat( _mat );
        return shifted_mat; // Early return 
    }

    MatrixXd shifted_mat = MatrixXd::Zero( _mat.rows(), _mat.cols() );
    for ( int col_idx = 0; col_idx < _mat.cols(); col_idx++ )
    {
        int new_location = (col_idx + _num_shift) % _mat.cols();
        shifted_mat.col(new_location) = _mat.col(col_idx);
    }

    return shifted_mat;

} // 

std::vector<float> eig2stdvec( MatrixXd _eigmat )//MatrixXd(VectorXd)->array
{
    std::vector<float> vec( _eigmat.data(), _eigmat.data() + _eigmat.size() );
    return vec;
} 

double SCManager::distDirectSC ( MatrixXd &_sc1, MatrixXd &_sc2 )
{//_sc1: curr          _sc2: candidate
    int num_eff_cols = 0; // i.e., to exclude all-nonzero sector
    double sum_sector_similarity = 0;
    for ( int id = 0; id < _sc1.cols(); id++ )
    {
        VectorXd col_sc1 = _sc1.col(id);
        VectorXd col_sc2 = _sc2.col(id);
        
        if( col_sc1.norm() == 0 | col_sc2.norm() == 0 )
            continue; // don't count this sector pair. 

        double sector_similarity = col_sc1.dot(col_sc2) / (col_sc1.norm() * col_sc2.norm());

        sum_sector_similarity = sum_sector_similarity + sector_similarity;
        num_eff_cols++;
    }
    
    double sc_sim = sum_sector_similarity / num_eff_cols;
    return 1.0 - sc_sim;

} // 

//vkey2从vkey1的哪一列开始展开--和vkey1的距离最小
int SCManager::fastAlignUsingVkey( MatrixXd & _vkey1, MatrixXd & _vkey2)
{//_vkey1: current     _vkey2: nearest neighbors
    int argmin_vkey_shift = 0;
    double min_veky_diff_norm = 10000000;
    for ( int idx = 0; idx < _vkey1.cols(); idx++ )
    {
        MatrixXd vkey2_shifted = circshift(_vkey2, idx);

        MatrixXd vkey_diff = _vkey1 - vkey2_shifted;

        double cur_diff_norm = vkey_diff.norm();
        if( cur_diff_norm < min_veky_diff_norm )
        {
            argmin_vkey_shift = idx;
            min_veky_diff_norm = cur_diff_norm;
        }
    }
    //"nearest neighbors"Matrix  从current的哪一列 与 currentMatrix距离最小,二者最类似
    return argmin_vkey_shift;

} //

//detectLoopClosureID调用
std::pair<double, int> SCManager::distanceBtnScanContext( MatrixXd &_sc1, MatrixXd &_sc2 )//cur candidate
{
    // 1. fast align using variant key (not in original IROS18)
    MatrixXd vkey_sc1 = makeSectorkeyFromScancontext( _sc1 );//current 1 * 60
    MatrixXd vkey_sc2 = makeSectorkeyFromScancontext( _sc2 );//nearest neighbors
    int argmin_vkey_shift = fastAlignUsingVkey( vkey_sc1, vkey_sc2 );//vkey_sc2从vkey_sc1的哪一列开始展开--和vkey_sc1的距离最小

    const int SEARCH_RADIUS = round( 0.5 * SEARCH_RATIO * _sc1.cols() ); // a half of search range 
    std::vector<int> shift_idx_search_space { argmin_vkey_shift };
    for ( int i = 1; i < SEARCH_RADIUS + 1; i++ )
    {
        shift_idx_search_space.push_back( (argmin_vkey_shift + i + _sc1.cols())   % _sc1.cols()   );
        shift_idx_search_space.push_back( (argmin_vkey_shift - i + _sc1.cols())   % _sc1.cols()   );
    }
    //argmin_vkey_shift双端拓展SEARCH_RADIUS
    //如果argmin_vkey_shift = 9 SEARCH_RADIUS=3, 则shift_idx_search_space存储 6->12
    std::sort(shift_idx_search_space.begin(), shift_idx_search_space.end());

    // 2. fast columnwise diff 
    int argmin_shift = 0;//current的哪一列(sector)开始与"nearest neighbors"的distance最小
    double min_sc_dist = 10000000;
    for ( int num_shift: shift_idx_search_space )
    {
        MatrixXd sc2_shifted = circshift(_sc2, num_shift);//_sc2从第num_shift列展开并返回新的matrix
        double cur_sc_dist = distDirectSC( _sc1, sc2_shifted );//curr即_sc1 和 candidate即sc2_shifted的距离,越小表示越相似
        if( cur_sc_dist < min_sc_dist )
        {
            argmin_shift = num_shift;
            min_sc_dist = cur_sc_dist;
        }
    }
    //(cur_sc和neighbor的最小距离,  neighbor从cur_sec的哪一列开始展开最接近cur_sec)
    return make_pair(min_sc_dist, argmin_shift);
} 


MatrixXd SCManager::makeScancontext( pcl::PointCloud<PointT> & _scan_down )
{
    TicToc t_making_desc;

    int num_pts_scan_down = _scan_down.points.size();

    // main
    const int NO_POINT = -1000;
    MatrixXd desc = NO_POINT * MatrixXd::Ones(PC_NUM_RING, PC_NUM_SECTOR);//20 60

    PointT pt;
    float azim_angle, azim_range; // wihtin 2d plane
    int ring_idx, sctor_idx;
    for (int pt_idx = 0; pt_idx < num_pts_scan_down; pt_idx++)
    {
        pt.x = _scan_down.points[pt_idx].x; 
        pt.y = _scan_down.points[pt_idx].y;
        pt.z = _scan_down.points[pt_idx].z + LIDAR_HEIGHT; // naive adding is ok (all points should be > 0).

        // xyz to ring, sector
        azim_range = sqrt(pt.x * pt.x + pt.y * pt.y);
        azim_angle = xy2theta(pt.x, pt.y);

        // if range is out of roi, pass
        if( azim_range > PC_MAX_RADIUS )//80m
            continue;
        int ring_id =  int(ceil( (azim_range / PC_MAX_RADIUS) * PC_NUM_RING ));
        ring_idx = std::max( std::min( PC_NUM_RING, ring_id), 1 );
        int sector_id = int(ceil( (azim_angle / 360.0) * PC_NUM_SECTOR ));
        sctor_idx = std::max( std::min( PC_NUM_SECTOR, sector_id), 1 );

        // taking maximum z 
        if ( desc(ring_idx-1, sctor_idx-1) < pt.z ) // -1 means cpp starts from 0
            desc(ring_idx-1, sctor_idx-1) = pt.z; // update for taking maximum value at that bin
    }

    // reset no points to zero (for cosine dist later)
    for ( int row_idx = 0; row_idx < desc.rows(); row_idx++ )
        for ( int col_idx = 0; col_idx < desc.cols(); col_idx++ )
            if( desc(row_idx, col_idx) == NO_POINT )
                desc(row_idx, col_idx) = 0;

    t_making_desc.toc("PolarContext making");

    return desc;
} // SCManager::makeScancontext


MatrixXd SCManager::makeRingkeyFromScancontext( Eigen::MatrixXd &_desc )
{
    /* 
     * summary: rowwise mean vector
    */
    Eigen::MatrixXd invariant_key(_desc.rows(), 1);
    for ( int row_idx = 0; row_idx < _desc.rows(); row_idx++ )
    {
        Eigen::MatrixXd curr_row = _desc.row(row_idx);
        invariant_key(row_idx, 0) = curr_row.mean();
    }

    return invariant_key;//20*1  20个ring,每个ring的mean value
} // SCManager::makeRingkeyFromScancontext


MatrixXd SCManager::makeSectorkeyFromScancontext( Eigen::MatrixXd &_desc )
{
    /* 
     * summary: columnwise mean vector
    */
    Eigen::MatrixXd variant_key(1, _desc.cols());
    for ( int col_idx = 0; col_idx < _desc.cols(); col_idx++ )
    {
        Eigen::MatrixXd curr_col = _desc.col(col_idx);
        variant_key(0, col_idx) = curr_col.mean();
    }

    return variant_key;
}  

//user's interface 1
void SCManager::makeAndSaveScancontextAndKeys( pcl::PointCloud<PointT> & _scan_down )
{
    Eigen::MatrixXd sc = makeScancontext(_scan_down); // v1, 20*60的image,每个像素存储最大高度值
    Eigen::MatrixXd ringkey = makeRingkeyFromScancontext( sc );//20*1  20个ring,每个ring的mean value
    Eigen::MatrixXd sectorkey = makeSectorkeyFromScancontext( sc );// 1*60 每个sector的mean value
    

    polarcontexts_.push_back( sc ); 
    polarcontext_invkeys_.push_back( ringkey );
    polarcontext_vkeys_.push_back( sectorkey );
    
    std::vector<float> polarcontext_invkey_vec = eig2stdvec( ringkey );//ringKey转换为 20*1的数组
    polarcontext_invkeys_mat_.push_back( polarcontext_invkey_vec );

    // cout <<polarcontext_vkeys_.size() << endl;

} // SCManager::makeAndSaveScancontextAndKeys

//user's interface 2
std::pair<int, float> SCManager::detectLoopClosureID ( void )
{
    int loop_id { -1 }; //loop's detection ID init with -1, -1 means no loop (== LeGO-LOAM's variable "closestHistoryFrameID")
    //curr_key为20*1的数组  polarcontext_invkeys_mat_为num_frames*20的二维数组
    auto curr_key = polarcontext_invkeys_mat_.back(); // current observation (query),mean value of current frame's ring(20*1)
    auto curr_desc = polarcontexts_.back(); // current observation (query)

    /* 
     * step 1: candidates from ringkey tree_
     */
    if( polarcontext_invkeys_mat_.size() < NUM_EXCLUDE_RECENT + 1)//NUM_EXCLUDE_RECENT 50
    {
        std::pair<int, float> result {loop_id, 0.0};
        return result; // Early return, too early
    }

    // tree_ reconstruction (not mandatory to make everytime)并非强制每次都重建搜索树tree
    if( tree_making_period_conter % TREE_MAKING_PERIOD_ == 0) // to save computation cost TREE_MAKING_PERIOD_=10 每次10次调用更新一次tree
    {
        TicToc t_tree_construction;

        polarcontext_invkeys_to_search_.clear();//二维数组
        //使用当前前NUM_EXCLUDE_RECENT=50帧之前的所有ringKey作为所搜集
        polarcontext_invkeys_to_search_.assign( polarcontext_invkeys_mat_.begin(), polarcontext_invkeys_mat_.end() - NUM_EXCLUDE_RECENT ) ;

        polarcontext_tree_.reset(); 
        polarcontext_tree_ = std::make_unique<InvKeyTree>(PC_NUM_RING /* dim */, polarcontext_invkeys_to_search_, 10 /* max leaf */ );//20 candidates 10
        // tree_ptr_->index_->buildIndex(); // inernally called in the constructor of InvKeyTree (for detail, refer the nanoflann and KDtreeVectorOfVectorsAdaptor)
        t_tree_construction.toc("Tree construction");
    }
    tree_making_period_conter = tree_making_period_conter + 1;
        
    double min_dist = 10000000; // init with somthing large
    int nn_align = 0;
    int nn_idx = 0;

    // knn search
    std::vector<size_t> candidate_indexes( NUM_CANDIDATES_FROM_TREE ); //NUM_CANDIDATES_FROM_TREE 10
    std::vector<float> out_dists_sqr( NUM_CANDIDATES_FROM_TREE );

    TicToc t_tree_search;
    nanoflann::KNNResultSet<float> knnsearch_result( NUM_CANDIDATES_FROM_TREE );
    knnsearch_result.init( &candidate_indexes[0], &out_dists_sqr[0] );
    polarcontext_tree_->index_->findNeighbors( knnsearch_result, &curr_key[0] /* query */, nanoflann::SearchParams(10) ); 
    t_tree_search.toc("Tree search");

    /* 
     *  step 2: pairwise distance (find optimal columnwise best-fit using cosine distance)
     */
    TicToc t_calc_dist;   
    for ( int candidate_iter_idx = 0; candidate_iter_idx < NUM_CANDIDATES_FROM_TREE; candidate_iter_idx++ )
    {
        MatrixXd polarcontext_candidate = polarcontexts_[ candidate_indexes[candidate_iter_idx] ];//邻近的scancontext
        std::pair<double, int> sc_dist_result = distanceBtnScanContext( curr_desc, polarcontext_candidate ); 
        
        double candidate_dist = sc_dist_result.first;//cur_sc和候选sc的距离
        int candidate_align = sc_dist_result.second;//候选sc从cur_sc的哪一列开始展开进行距离的计算

        if( candidate_dist < min_dist )
        {
            min_dist = candidate_dist;//distance
            nn_align = candidate_align;//col's index of current

            nn_idx = candidate_indexes[candidate_iter_idx];//id of nearest's image
        }
    }
    t_calc_dist.toc("Distance calc");

    /* 
     * loop threshold check
     */
    if( min_dist < SC_DIST_THRES )
    {
        loop_id = nn_idx; //id of nearest's image 历史帧中闭环点scan的id
    
        // std::cout.precision(3); 
        cout << "[Loop found] Nearest distance: " << min_dist << " btn " << polarcontexts_.size()-1 << " and " << nn_idx << "." << endl;
        cout << "[Loop found] yaw diff: " << nn_align * PC_UNIT_SECTORANGLE << " deg." << endl;
    }
    else
    {
        std::cout.precision(3); 
        cout << "[Not loop] Nearest distance: " << min_dist << " btn " << polarcontexts_.size()-1 << " and " << nn_idx << "." << endl;
        cout << "[Not loop] yaw diff: " << nn_align * PC_UNIT_SECTORANGLE << " deg." << endl;
    }

    // To do: return also nn_align (i.e., yaw diff)
    float yaw_diff_rad = deg2rad(nn_align * PC_UNIT_SECTORANGLE);
    std::pair<int, float> result {loop_id, yaw_diff_rad};//(历史帧中闭环点scan的id, 闭环帧在当前帧起点的角度) 

    return result;//id of nearest's image ---- current's start angle

} // end detectLoopClosureID

// } // namespace SC2