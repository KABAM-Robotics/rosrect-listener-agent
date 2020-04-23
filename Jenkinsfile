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
            def script_bash = libraryResource '/opt/ros/melodic/setup.bash'
            writeFile file: './test.sh', text: script_bash

            sh 'bash ./test.sh'

      }
    }

  }
}
