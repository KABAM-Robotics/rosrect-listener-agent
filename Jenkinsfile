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
            sh '''
            sh coverage.sh
            '''
        
      cobertura coberturaReportFile: 'coverage.xml'
    }
    }
  }
  post {
  always {
      step([$class: 'CoberturaPublisher', autoUpdateHealth: false, autoUpdateStability: false, coberturaReportFile: '**/coverage.xml', failUnhealthy: false, failUnstable: false, maxNumberOfBuilds: 0, onlyStable: false, sourceEncoding: 'ASCII', zoomCoverageChart: false])
  }
}
}
