pipeline{
  agent { label 'ubuntu' }
  stages{
    stage('--build--'){
      steps{
            echo 'Building and running the tests'
            sh '''
            sh test.sh
            '''
      }
    }
    stage('--CD--'){
      steps{
            echo 'Deploying the agents'
            sh '''
            sh cd.sh
            '''
      }
    }
    stage ("--Extract test results--") {
    steps {
            echo 'Estimating code coverage'
            //sh '''
            //sh coverage.sh
            //'''
        
      //cobertura coberturaReportFile: 'coverage.xml'
    }
    }
  }
  post {
  always {
    slackSend(message:"Job ${env.JOB_NAME} ${env.BUILD_NUMBER} with commit: '${env.GIT_COMMIT}' has completed with status: ${currentBuild.currentResult} \n (<${env.BUILD_URL}|Open>)")
  }
}
}
