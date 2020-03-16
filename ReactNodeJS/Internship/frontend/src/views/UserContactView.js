"use strict";

import React from 'react';
import ReactLoading from 'react-loading';

import UserContact from '../components/UserContact';
import ProjectService from '../services/ProjectService';
import CompanyService from '../services/CompanyService';
import UserService from '../services/UserService';

export class UserContactView extends React.Component {

    constructor(props) {
        super(props);
        this.state = {
            loading: true,
            user: this.props.history.location.state.user,
            company: UserService.getCurrentUser(),
            projects: []
        }
    }

    componentWillMount(props)
    {
        let query = {
            "query" : {
                "term" : {"company.id": this.state.company.id}
            }
        };
        ProjectService.search(query).then((data) => {
            this.setState({
                projects: data.hits.hits,
                loading: false
            });
        }).catch((e) => {
            console.error(e);
        });
    }

    sendMail(data) {
        this.setState({
            loading: true
        }, () => CompanyService.sendMail(data).then((result) => {
            console.log(result);
            this.props.history.push('/');
        }).catch((e) => {
            console.log(e);
            this.setState({
                loading: false,
                error: e
            });
        }));
    }

    render() {
        if (this.state.loading) {
            return (
                <div style={{marginTop:200+'px'}}>
                    <div className="col-md-5"/>
                    <div className="col-md-2">
                        <ReactLoading type="spinningBubbles" color="#000000" height={500} width={200}/>
                    </div>
                    <div className="col-md-5"/>
                </div>
            );
        }

        return (
            <UserContact user={this.state.user} company={this.state.company} projects={this.state.projects} onMessageSubmit={(data) => this.sendMail(data)}/>
        );
    }
}