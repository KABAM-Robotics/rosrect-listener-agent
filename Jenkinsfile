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
        def exists = fileExists 'rosrect-listener-agent'
        if (exists) {
            echo 'Dir Already exists'
            } else {
            sh 'git clone https://github.com/cognicept-admin/rosrect-listener-agent.git'
            }
            cd rosrect-listener-agent
            sh 'catkin_make run_tests_rosrect-listener-agent'
      }
    }

  }
}
