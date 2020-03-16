"use strict";

import React from 'react';
import ReactLoading from 'react-loading';

import CompanyOnboarding from '../components/CompanyOnboarding';

import UserService from '../services/UserService';


export class CompanyOnboardingView extends React.Component {

    constructor(props) {
        super(props);
    }

    componentWillMount(props)
    {
        this.setState({
            loading: false
        });
    }

    requestUpgrade(user) {
        UserService.requestUpgrade(user).then((data) => {
            this.setState({
                didSubmit: true
            })
        }).catch((e) => {
            console.error(e);
            this.setState({
                error: e
            });
        });
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
            <CompanyOnboarding user={this.state.user} didSubmit={this.state.didSubmit} onSubmit={(user) => this.requestUpgrade(user)} error={this.state.error}/>
        );
    }
}