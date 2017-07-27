

//---PolicyVisualizer--------//
//PolicyVisualizer::PolicyVisualizer( const Policy::ptr & policy, const std::string & name )
//{
//  views_.resize( policy->N() );
//  for( auto w = 0; w < policy->N(); ++w )
//  {
//    std::string windowName = name + std::string( "-world-" ) + std::to_string( w );
//    views_[ w ] = std::make_shared< OrsPathViewer >( windowName.c_str(),  0.1, -0 );
//    views_[ w ]->setConfigurations( policy->getTrajectory( w ) );
//  }


//  threadOpenModules( true );
//}
