"use strict";

import React from 'react';
import {withRouter} from 'react-router-dom'
import { Card, CardTitle, CardText, Media, MediaOverlay, Grid, Cell, Button, FontIcon } from 'react-md';
import Page from './Page';
import {Image} from "react-bootstrap";
import imageProject from "../assets/img/projectImage.png";
import CompanySearchResults from "./CompanySearchResults";
import MatchingResults from "./MatchingResults";
import UserService from "../services/UserService";
import {Footer} from "./Footer";

const style = {height: "100vh",backgroundColor:"#FFF2D6"};

export class ProjectDetails extends React.Component {

    constructor(props) {
        super(props);

        this.state = {
            users: [],
            scores:[],
            loggedInUser: UserService.isAuthenticated() ? UserService.getCurrentUser() : undefined
        }
    }

    render() {
        let project = this.props.project._source;
        let projectId = this.props.project._id;
        let startDate = "";
        let endDate = "";
        if(startDate){
           startDate = project.timeFrame.startDate.substring(8, 10) +'/' + project.timeFrame.startDate.substring(5, 7) +'/' + project.timeFrame.startDate.substring(0, 4);
        }
        if(endDate){
            endDate = project.timeFrame.endDate.substring(8, 10) +'/' + project.timeFrame.endDate.substring(5, 7) +'/' + project.timeFrame.endDate.substring(0, 4);
        }
        if(project && project.recommendations){
            this.users = project.recommendations.matches;
        }
        /*<li> Duration: <FormattedDate value={project.timeFrame.startDate}/> - <FormattedDate value={project.timeFrame.endDate}/> </li> */
        return (
            <Page isStickyFooter={true}>
                <div className= "row md-block-centered" style={style}>
                    <div className ="container-fluid">
                        <Image classname = "center-block" src={imageProject} style={{display: "block", margin:"0 auto", width:"90%", height:"275"}}/>
                    </div>
                    <div className="col-md-1"> </div>
                    <div className="col-md-7"  style={{marginTop:'4em'}}>
                        <div className= " text-left Project" style={{ fontWeight: 'bold', padding:'30'}} >
                            <h2 style = {{fontSize :"2" + "em", fontWeight: 'bold'}}>{project.title}</h2>
                            <h3>Project Offerer: {project.company.name}</h3>
                            <h4 style={{marginTop:'3em'}}>{project.overview}</h4>
                            <h4 style={{marginTop:'3em'}}>{project.responsibilities}</h4>
                        </div>
                    </div>
                    <div className="Facts col-md-2 "  style={{marginTop:'6em', color:"grey", marginLeft: '3em'}}>
                        <ul> Facts:
                            <li style={{marginTop:"1em"}}>Project Location: {project.location}</li>
                            <li>Project Language: {project.language}</li>
                            <li> Duration: {startDate} - {endDate} </li>
                            <li>Team Set-up:  {project.teamSetup}</li>
                        </ul>
                        {this.state.loggedInUser && this.state.loggedInUser.role === "company" && this.state.loggedInUser.id===this.props.project._source.company.id ? [
                                <div className="matchButton text-center">
                                    {project.recommendations ? [
                                            <Button style={{paddingBottom:"2.5em"}} raised secondary iconBefore={false} onClick={() => this.props.history.push({pathname: `/company/recommendations/${projectId}`, state: {users: this.users}})}>Best matches for this project</Button>
                                        ]
                                        :
                                        [
                                            <h2>No Matches Available for this Project</h2>
                                        ]
                                    }
                                </div>
                            ]
                            :
                            []
                        }
                        <div className="col-md-2"> </div>
                    </div>
                </div>
            </Page>
        );
    }
}
export default withRouter(ProjectDetails);
3