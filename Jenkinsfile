pipeline{
  agent { label 'ubuntu' }
  stages{
    stage('--test--'){
      steps{
        echo 'conducting tests'
      }
    }
    stage('--build--'){
      steps{
            sh 'mkdir -p catkin_ws_rosrect_listener/src'
            sh 'cd catkin_ws_rosrect_listener/src'
            sh 'git clone https://github.com/cognicept-admin/rosrect-listener-agent.git'
            sh 'cd rosrect-listener-agent'
            sh 'git checkout feature/state-manager-revamp'
            sh 'cd .. && cd ..'
            sh 'catkin_make run_tests_rosrect-listener-agent'
      }
    }

  }
}
